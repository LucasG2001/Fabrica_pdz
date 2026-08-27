"""Generate gz-sim SDF models for the plumbers_block fixture + 5 parts, from Fabrica's own
assembly meshes and this specific plan's fixture/pickup poses.

Run in the `fabrica` conda env (needs trimesh, numpy, scipy):
    conda run -n fabrica python planning/utils/generate_plumbers_block_gz_assets.py

Inputs (Fabrica's cm-scale board frame, arm pair midpoint at Y=10cm per
planning/robot/workcell.py's get_move_arm_pos('kuka') -- see the approved plan in
~/.claude/plans/wise-prancing-cocke.md, "Key findings" #2 and #4):
  - assets/fabrica/plumbers_block/{0..4}.obj: per-part visual meshes.
  - logs/plumbers_block_sim/fixture/fixture.obj: fixture body mesh, already exported at the
    correct board-frame position (bounds cross-checked against validation.json's "fixture"
    footprint -- no extra transform needed).
  - logs/plumbers_block_sim/fixture/pickup.json: per-part pose {x,y,z,rx,ry,rz} (cm, xyz-euler
    rad), already in the same absolute board frame as pickup.json (cross-checked against
    validation.json's pickup_part_N footprints).

Output: one gz-sim model (model.config + model.sdf + meshes/) per part and per the fixture,
under <out_dir>/models/. Visual geometry uses the original mesh. Collision geometry is
per-part (see COLLISION_BUILDERS below, and build_fixture_mesh_collision for the fixture):
- fixture: full real mesh (reuses the exported visual.obj as-is), kept as exact geometry per
  explicit user direction 2026-08-25, not approximated as a box, even though it's static and a
  box would be cheaper/safer -- see this file's inline history comments for the tunneling
  incident this caused before the fixes below, and don't revert this to a box without asking.
- pipe, and any part without an override: a single AABB box (build_default_box_collision) --
  fine for simple blocky shapes.
- screws: a single thin cylinder along the long axis (build_screw_collision) -- a two-piece
  head-box + shaft-cylinder compound was tried first and made things visibly worse; simplified
  to one primitive per user direction.
- base/top: SDF-native `<mesh><convex_decomposition>` (build_convex_decomposition_collision) --
  their geometry is too complex for a box, and gz-sim/DART decomposing it internally (rather than
  us pre-decomposing via trimesh/VHACD and shipping N hull files, which silently failed to
  generate any contact against the fixture -- see history comments) gives real convex collision
  objects that contact the fixture mesh correctly.
All parts also get PART_MASS_KG-based mass, a per-part box_inertia() tensor (NOT a flat
placeholder -- see that function's docstring for the solver-explosion this fixes), and bumped
friction/damping (COLLISION_FRICTION_MU, LINK_LINEAR_DAMPING, LINK_ANGULAR_DAMPING) so parts
settle solidly instead of floating/sliding. All poses/meshes are converted from Fabrica's
centimeters to gz-sim's meters.

Known unresolved issue (2026-08-25): with all of the above, 4 of 5 dynamic parts (base, top,
pipe, screw_a) settle within ~1mm of their intended spawn_poses.json pose, but screw_b
consistently settles ~5cm off (stable, not still falling/sliding -- friction/damping are working
-- just at the wrong location). screw_a and screw_b use identical collision/mass/friction and
the same (real, mesh) fixture, so this looks like an actual asymmetry in the fixture mesh's
geometry near screw_b's slot specifically, not a config/collision-shape-choice bug elsewhere in
this file. Not investigated further this session (user decision) -- see
kuka_pdz_gazebo_collision_tuning_handoff.md for what was tried and what a future session should
check first (careful: any vertex-proximity check on fixture.obj needs the same board frame
transform pose_from_pickup()/board_transform_matrix() apply elsewhere in this file -- a first
attempt at this check queried pickup.json's raw board-frame (x,y) directly against fixture.obj's
raw vertices with no transform and found "0 nearby vertices" for *both* screws, which is a sign
of a frame mismatch in the check itself, not a real finding -- redo it correctly rather than
trusting that result).
"""
import argparse
import os
import shutil
import sys

import numpy as np
import trimesh
from scipy.spatial.transform import Rotation

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..')))
from planning.robot.workcell import get_move_arm_pos

CM_TO_M = 0.01

# part_id -> (semantic name, rgba color) -- user-confirmed mapping, see the approved plan.
PART_INFO = {
    '2': ('base', (0.15, 0.55, 0.20, 1.0)),   # green
    '3': ('top', (0.95, 0.80, 0.10, 1.0)),    # yellow
    '0': ('pipe', (0.45, 0.28, 0.13, 1.0)),   # brown
    '1': ('screw_a', (0.80, 0.10, 0.10, 1.0)),  # red
    '4': ('screw_b', (0.80, 0.10, 0.10, 1.0)),  # red
}
FIXTURE_COLOR = (0.55, 0.55, 0.55, 1.0)

# part_id -> mass (kg). Bumped up from a uniform 50g (2026-08-25, user direction) so parts settle
# more solidly instead of floating/drifting after contact -- heavier parts also get proportionally
# larger box_inertia() tensors, which further helps contact stability (see box_inertia's docstring
# for the small-inertia solver-explosion history this interacts with).
PART_MASS_KG = {
    '2': 0.150,  # base
    '3': 0.080,  # top
    '0': 0.100,  # pipe
    '1': 0.050,  # screw_a
    '4': 0.050,  # screw_b
}

# Surface friction + link velocity damping, both bumped up 2026-08-25 (user direction) so parts
# stop sliding/floating around after settling. mu/mu2 well above the ~1.0 default; velocity_decay
# is a small extra damping term on top of contact friction alone.
COLLISION_FRICTION_MU = 1.6
LINK_LINEAR_DAMPING = 0.05
LINK_ANGULAR_DAMPING = 0.05

SURFACE_XML = (
    '        <surface>\n'
    '          <friction>\n'
    '            <ode><mu>{mu}</mu><mu2>{mu}</mu2></ode>\n'
    '          </friction>\n'
    '        </surface>\n'
).format(mu=COLLISION_FRICTION_MU)

MODEL_CONFIG_TEMPLATE = """<?xml version="1.0"?>
<model>
  <name>{name}</name>
  <version>1.0</version>
  <sdf version="1.9">model.sdf</sdf>
  <description>Auto-generated from Fabrica's plumbers_block assets ({source}).</description>
</model>
"""

MODEL_SDF_TEMPLATE = """<?xml version="1.0"?>
<sdf version="1.9">
  <model name="{name}">
    <static>{static}</static>
    <link name="link">
      <inertial>
        <mass>{mass}</mass>
        <inertia>
          <ixx>{ixx}</ixx><iyy>{iyy}</iyy><izz>{izz}</izz>
          <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz>
        </inertia>
      </inertial>
      <velocity_decay>
        <linear>{linear_damping}</linear>
        <angular>{angular_damping}</angular>
      </velocity_decay>
      <visual name="visual">
        <geometry>
          <mesh><uri>meshes/visual.obj</uri><scale>{scale} {scale} {scale}</scale></mesh>
        </geometry>
        <material>
          <ambient>{r} {g} {b} {a}</ambient>
          <diffuse>{r} {g} {b} {a}</diffuse>
        </material>
      </visual>
{collision_xml}    </link>
  </model>
</sdf>
"""


def build_box_collision_xml(name, center, size, rpy=(0.0, 0.0, 0.0)):
    return (
        f'      <collision name="{name}">\n'
        f'        <pose>{center[0]} {center[1]} {center[2]} {rpy[0]} {rpy[1]} {rpy[2]}</pose>\n'
        f'        <geometry>\n'
        f'          <box><size>{max(size[0], 1e-4)} {max(size[1], 1e-4)} {max(size[2], 1e-4)}</size></box>\n'
        f'        </geometry>\n'
        f'{SURFACE_XML}'
        f'      </collision>\n'
    )


def build_cylinder_collision_xml(name, center, radius, length, rpy=(0.0, 0.0, 0.0)):
    return (
        f'      <collision name="{name}">\n'
        f'        <pose>{center[0]} {center[1]} {center[2]} {rpy[0]} {rpy[1]} {rpy[2]}</pose>\n'
        f'        <geometry>\n'
        f'          <cylinder><radius>{radius}</radius><length>{length}</length></cylinder>\n'
        f'        </geometry>\n'
        f'{SURFACE_XML}'
        f'      </collision>\n'
    )


def build_mesh_collision_xml(name, mesh_uri):
    return (
        f'      <collision name="{name}">\n'
        f'        <geometry>\n'
        f'          <mesh><uri>{mesh_uri}</uri><scale>1 1 1</scale></mesh>\n'
        f'        </geometry>\n'
        f'{SURFACE_XML}'
        f'      </collision>\n'
    )


def build_default_box_collision(mesh_m, mesh_dir):
    """Single AABB box in the part's own (already-meters) local frame -- fine for blocky/simple
    shapes (also what the fixture uses, and any part without a COLLISION_BUILDERS override)."""
    center = mesh_m.bounds.mean(axis=0)
    extents = mesh_m.bounds[1] - mesh_m.bounds[0]
    return build_box_collision_xml('collision', center, extents)


def build_screw_collision(mesh_m, mesh_dir):
    """Single thin cylinder spanning the screw's full long axis (auto-detected), radius 4.6mm.
    A two-piece head-box + shaft-cylinder compound was tried first (sized from the actual mesh
    geometry) but made things worse in practice (2026-08-25) -- reverted to this single-primitive
    approximation per user direction. A plain AABB box (the very first attempt) was already known
    bad: it wraps the wide head around the thin shaft too, so in the fixture's tight screw holes
    the oversized box interpenetrates walls the real shaft wouldn't touch and physics kicks the
    screw out of its seated pose on spawn."""
    axis = int(np.argmax(mesh_m.extents))
    other_axes = [i for i in range(3) if i != axis]
    lo, hi = mesh_m.bounds[0, axis], mesh_m.bounds[1, axis]
    length = hi - lo
    perp_center = mesh_m.bounds[:, other_axes].mean(axis=0)

    center = [0.0, 0.0, 0.0]
    center[axis] = (lo + hi) / 2
    for i, oa in enumerate(other_axes):
        center[oa] = perp_center[i]

    # SDF <cylinder> is aligned along the collision frame's local Z by default; rotate onto
    # whichever axis is actually the screw's long axis.
    cylinder_rpy = {2: (0.0, 0.0, 0.0), 0: (0.0, np.pi / 2, 0.0), 1: (-np.pi / 2, 0.0, 0.0)}[axis]
    return build_cylinder_collision_xml('collision', center, 0.0046, length, cylinder_rpy)


def build_fixture_mesh_collision(mesh_m, mesh_dir):
    """Full mesh collision, reusing the already-exported visual.obj as-is. Static (the fixture
    doesn't move), per user direction 2026-08-25: keep the fixture's real geometry (its
    cavities/features are what parts actually seat against) rather than any box/primitive
    approximation, even though a static mesh collider isn't literally required for stability the
    way it is for a moving body."""
    return build_mesh_collision_xml('collision', 'meshes/visual.obj')


def build_convex_decomposition_collision(mesh_m, mesh_dir, max_hulls=8):
    """SDF-native convex decomposition: a single <mesh> collision (reusing visual.obj) with a
    <convex_decomposition> hint, so gz-sim/DART decomposes and treats the pieces as real convex
    collision objects internally. base/top's geometry is too complex for a box or a couple of
    primitives (both were mis-seated by the single AABB-box default -- confirmed via the same
    spawn-vs-settled pose check used for the screws).

    History (2026-08-25): first tried doing the decomposition ourselves via trimesh/VHACD and
    shipping N separate hull .obj files, each declared as a plain <mesh> collision -- gz-sim
    didn't crash or tank real_time_factor on that, but contact generation against the fixture
    silently failed: base/top free-fell straight through the fixture *and* the table/ground
    (settled z around -1.6m, confirmed via pose telemetry), while primitive collisions (the
    screw's cylinder, the pipe's box) against the same fixture worked fine. Root cause suspected:
    a plain <mesh> element is parsed as an arbitrary triangle mesh (BVH-based collision), not a
    true convex primitive, so mesh-vs-mesh narrowphase from an already-overlapping spawn state
    apparently generates no contact. This <convex_decomposition> tag is the fix attempt: it's
    documented sdformat/gz-sim behavior (not just a VHACD preprocessing hint) that should make
    DART treat the hulls as real convex collision objects, giving them the same convex-vs-mesh
    contact path that already works for the screw/pipe primitives. If gz-sim's build doesn't
    support this tag (silently ignored) or it still doesn't generate contacts, fall back to
    build_default_box_collision for base/top (COLLISION_BUILDERS) rather than debugging further
    live."""
    return (
        '      <collision name="collision">\n'
        '        <geometry>\n'
        '          <mesh>\n'
        '            <uri>meshes/visual.obj</uri>\n'
        '            <convex_decomposition>\n'
        f'              <max_convex_hulls>{max_hulls}</max_convex_hulls>\n'
        '            </convex_decomposition>\n'
        '          </mesh>\n'
        '        </geometry>\n'
        f'{SURFACE_XML}'
        '      </collision>\n'
    )


# part_id -> collision builder override (falls back to build_default_box_collision -- a plain
# AABB box, used by the pipe and any part not listed here). See each builder's docstring for why.
COLLISION_BUILDERS = {
    '1': build_screw_collision,      # screw_a
    '4': build_screw_collision,      # screw_b
    '2': build_convex_decomposition_collision,  # base
    '3': build_convex_decomposition_collision,  # top
}


# Fabrica's board frame separates the arms along X and offsets the assembly along Y
# (get_move_arm_pos/get_hold_arm_pos = (+-42, 10, 2.5) cm, get_assembly_center = (0, -15, 0) cm);
# lbr_dual_arm.xacro's actual base joints separate the arms along Y instead, both at X=0. Spawning
# Fabrica's raw (x,y) directly therefore puts things next to one arm instead of out in front of
# the pair. Fix: rotate by +90 deg about Z around the arm-pair midpoint (0, get_move_arm_pos('kuka')[1])
# = (0, 10) cm -- verified against both arm base joints: move arm (42,10)->gazebo (0,0.42)m
# matches lbr_two_base_joint, hold arm (-42,10)->gazebo (0,-0.42)m matches lbr_one_base_joint.
# Computed live (rather than hardcoded) so this can't drift out of sync with workcell.py again
# like it did 2026-08-24 (Y changed 8*dx=20 -> 10, see workcell.get_move_arm_pos's kuka branch).
BOARD_MIDPOINT_CM = np.array([0.0, get_move_arm_pos('kuka')[1], 0.0])
BOARD_TO_GAZEBO_ROT = Rotation.from_euler('z', 90, degrees=True)


def pose_from_pickup(entry_cm_rad):
    x, y, z, rx, ry, rz = entry_cm_rad
    pos_cm_rel = np.array([x, y, z]) - BOARD_MIDPOINT_CM
    pos_m = BOARD_TO_GAZEBO_ROT.apply(pos_cm_rel) * CM_TO_M
    orientation = BOARD_TO_GAZEBO_ROT * Rotation.from_euler('xyz', [rx, ry, rz])
    quat_xyzw = orientation.as_quat()
    rpy = orientation.as_euler('xyz')
    return pos_m, quat_xyzw, rpy


def board_transform_matrix():
    """4x4 transform taking Fabrica board-frame mesh vertices (cm, absolute) directly to
    Gazebo world-frame meters -- same +90 deg-about-Z-about-the-arm-pair-midpoint rotation as
    pose_from_pickup, but baked into the mesh itself (used for the fixture, whose mesh is
    exported pre-placed at its absolute board-frame position rather than at a local origin)."""
    R = BOARD_TO_GAZEBO_ROT.as_matrix()
    M = np.eye(4)
    M[:3, :3] = CM_TO_M * R
    M[:3, 3] = -CM_TO_M * (R @ BOARD_MIDPOINT_CM)
    return M


def box_inertia(mass, extents):
    """Solid-box inertia tensor (diagonal) from a part's own AABB extents and mass. Replaces a
    prior hardcoded ixx=iyy=izz=1e-5 for every part regardless of size, which was unrealistically
    small for the larger parts (e.g. base: 16x4x6.5cm) -- with box/cylinder collision primitives
    this apparently stayed stable enough not to matter, but once collision switched to mesh-based
    shapes (more simultaneous contact points per part) it fed a physics-solver explosion (fixture
    falling through the table, parts flying, even the unrelated robot grippers destabilizing --
    observed 2026-08-25). A small floor avoids a literal zero for near-planar extents."""
    dx, dy, dz = extents
    ixx = mass / 12.0 * (dy ** 2 + dz ** 2)
    iyy = mass / 12.0 * (dx ** 2 + dz ** 2)
    izz = mass / 12.0 * (dx ** 2 + dy ** 2)
    floor = 1e-9
    return max(ixx, floor), max(iyy, floor), max(izz, floor)


def write_model(out_dir, name, source_obj, rgba, static, mass=0.05,
                 board_frame=False, collision_builder=None):
    model_dir = os.path.join(out_dir, 'models', name)
    mesh_dir = os.path.join(model_dir, 'meshes')
    os.makedirs(mesh_dir, exist_ok=True)

    mesh = trimesh.load(source_obj, force='mesh')
    mesh_m = mesh.copy()
    if board_frame:
        mesh_m.apply_transform(board_transform_matrix())
    else:
        mesh_m.apply_scale(CM_TO_M)
    mesh_m.fix_normals()
    mesh_m.export(os.path.join(mesh_dir, 'visual.obj'), include_normals=True)

    collision_builder = collision_builder or build_default_box_collision
    collision_xml = collision_builder(mesh_m, mesh_dir)
    ixx, iyy, izz = box_inertia(mass, mesh_m.extents)

    with open(os.path.join(model_dir, 'model.config'), 'w') as f:
        f.write(MODEL_CONFIG_TEMPLATE.format(name=name, source=os.path.basename(source_obj)))

    with open(os.path.join(model_dir, 'model.sdf'), 'w') as f:
        f.write(MODEL_SDF_TEMPLATE.format(
            name=name, static='true' if static else 'false',
            scale=1.0, mass=mass, ixx=ixx, iyy=iyy, izz=izz,
            linear_damping=LINK_LINEAR_DAMPING, angular_damping=LINK_ANGULAR_DAMPING,
            r=rgba[0], g=rgba[1], b=rgba[2], a=rgba[3],
            collision_xml=collision_xml))

    return model_dir


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--fabrica-dir', default=os.path.expanduser('~/Fabrica'))
    parser.add_argument('--log-dir', default=None, help='defaults to <fabrica-dir>/logs/plumbers_block_sim')
    parser.add_argument('--out-dir', default=os.path.expanduser(
        '~/franka_ros2_ws/src/plumbers_block_description'))
    args = parser.parse_args()

    log_dir = args.log_dir or os.path.join(args.fabrica_dir, 'logs', 'plumbers_block_sim')
    assembly_dir = os.path.join(args.fabrica_dir, 'assets', 'fabrica', 'plumbers_block')

    if os.path.isdir(os.path.join(args.out_dir, 'models')):
        shutil.rmtree(os.path.join(args.out_dir, 'models'))

    import json
    with open(os.path.join(log_dir, 'fixture', 'pickup.json')) as f:
        pickup = json.load(f)

    poses = {}

    # Fixture: static, mesh already at its correct board-frame position (verified against
    # validation.json's "fixture" footprint) -- the board->gazebo rotation is baked into the
    # mesh itself (board_frame=True) rather than applied as a spawn pose, then spawned identity.
    fixture_obj = os.path.join(log_dir, 'fixture', 'fixture.obj')
    write_model(args.out_dir, 'plumbers_block_fixture', fixture_obj, FIXTURE_COLOR, static=True,
                board_frame=True, collision_builder=build_fixture_mesh_collision)
    poses['plumbers_block_fixture'] = (np.zeros(3), np.array([0.0, 0.0, 0.0, 1.0]), np.zeros(3))

    # Parts: dynamic (movable -- the orchestrator teleports a held part to match Fabrica's own
    # traj.npy pose for that frame while it's being carried; see plan_executor_node.py), spawned
    # at their pickup.json pose.
    for part_id, (semantic_name, rgba) in PART_INFO.items():
        model_name = f'plumbers_block_part{part_id}_{semantic_name}'
        source_obj = os.path.join(assembly_dir, f'{part_id}.obj')
        write_model(args.out_dir, model_name, source_obj, rgba, static=False,
                    mass=PART_MASS_KG[part_id],
                    collision_builder=COLLISION_BUILDERS.get(part_id))
        poses[model_name] = pose_from_pickup(pickup[part_id])

    poses_path = os.path.join(args.out_dir, 'spawn_poses.json')
    with open(poses_path, 'w') as f:
        json.dump({name: {'pos': pos.tolist(), 'quat_xyzw': quat.tolist(), 'rpy': rpy.tolist()}
                   for name, (pos, quat, rpy) in poses.items()}, f, indent=2)

    print(f'Wrote {len(poses)} models to {args.out_dir}/models')
    print(f'Wrote spawn poses to {poses_path}')


if __name__ == '__main__':
    main()
