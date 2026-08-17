import os
import sys

project_base_dir = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..'))
sys.path.append(project_base_dir)
os.environ['OMP_NUM_THREADS'] = '1'

import pickle
import numpy as np
import trimesh

from planning.run_grasp_arm_gen import GraspArmGenerator
from planning.robot.geometry import get_gripper_open_ratio, get_gripper_meshes_transforms, load_arm_meshes, transform_arm_meshes
from planning.robot.util_arm import get_arm_chain, get_ik_target_orientation, get_ft_pos_from_gripper_pos_quat, get_gripper_pos_quat_from_arm_q
from planning.robot.util_grasp import compute_antipodal_pairs, generate_gripper_states, Grasp
from planning.config import RETRACT_OPEN_RATIO


def build_candidates(gen, part_id):
    part_mesh = gen.part_meshes[part_id].copy()
    part_mesh.apply_transform(gen.part_final_transforms[part_id])

    antipodal_pairs = compute_antipodal_pairs(part_mesh, sample_budget=gen.n_surface_pt, antipodal_thres=gen.antipodal_thres, collision_meshes=[])
    grasps_cand = []
    grasp_id = 0
    for antipodal_points in antipodal_pairs:
        open_ratio = get_gripper_open_ratio(gen.gripper_type, antipodal_points)
        if open_ratio is None or open_ratio > 0.95:
            continue
        gripper_pos_list, gripper_quat_list = generate_gripper_states(gen.gripper_type, antipodal_points, open_ratio, gen.n_angle, offset_delta=gen.offset_delta)
        for gripper_pos, gripper_quat in zip(gripper_pos_list, gripper_quat_list):
            grasps_cand.append(Grasp(part_id, grasp_id, gripper_pos, gripper_quat, open_ratio))
            grasp_id += 1
    return part_mesh, grasps_cand


def is_ground_collision(gen, grasp):
    retract_open_ratio = min(grasp.open_ratio + RETRACT_OPEN_RATIO, 1.0)
    for open_ratio in [grasp.open_ratio, retract_open_ratio]:
        gt = get_gripper_meshes_transforms(gen.gripper_type, gen.gripper_meshes, grasp.pos, grasp.quat, np.eye(4), min(open_ratio + 0.05, 1.0))
        gen.apply_transforms_to_col_manager(gen.gripper_col_manager_buffered, gt)
        if gen.gripper_col_manager_buffered.in_collision_other(gen.ground_col_manager):
            return True
    return False


def is_self_collision(gen, grasp, part_id):
    gen.apply_transforms_to_col_manager(gen.part_col_manager, gen.part_final_transforms)
    gt = get_gripper_meshes_transforms(gen.gripper_type, gen.gripper_meshes, grasp.pos, grasp.quat, np.eye(4), min(grasp.open_ratio + 0.05, 1.0))
    gen.apply_transforms_to_col_manager(gen.gripper_col_manager, gt)
    _, contact_data = gen.gripper_col_manager.in_collision_other(gen.part_col_manager, return_data=True)
    return any(part_id in cdata.names for cdata in contact_data)


def is_zero_contact(gen, grasp, part_id):
    g = gen.compute_contact_points(grasp, part_id)
    return len(g.contact_points) == 0


FAILURE_MODES = {
    'ground-collision': is_ground_collision,
    'self-collision': lambda gen, grasp: is_self_collision(gen, grasp, grasp.part_id),
    'zero-contact': lambda gen, grasp: is_zero_contact(gen, grasp, grasp.part_id),
    'success': lambda gen, grasp: gen.check_grasp_feasible(grasp, grasp.part_id, verbose=False) is not None,
}


def compute_arm_pose(gen, grasp, motion_type='move'):
    '''
    Same IK call (and the same post-hoc FK verification) check_grasp_feasible makes at timestep 0
    for the given motion_type, so the rendered arm pose -- and whether it's reported as feasible
    -- matches what the real feasibility check would do.

    NOTE: `arm_chain.inverse_kinematics_above_ground`'s own `success` flag only checks that the
    resulting joints stay above the ground plane -- it does NOT check that the solver actually
    converged near the target (least_squares can return a poor local optimum and still report
    "success"). check_grasp_feasible additionally re-runs FK and compares against the requested
    gripper pose (`np.allclose(...)`); skipping that here would silently render nonsense arm
    poses (e.g. the end effector tens of cm from the intended grasp) as if they were valid.
    '''
    arm_chain = gen.arm_chains[motion_type]
    gripper_ori = get_ik_target_orientation(arm_chain, gen.gripper_type, grasp.quat)
    ft_pos = get_ft_pos_from_gripper_pos_quat(gen.gripper_type, grasp.pos, grasp.quat)
    target_position = ft_pos if gen.has_ft_sensor[motion_type] else grasp.pos
    arm_q_default = arm_chain.active_to_full(arm_chain.rest_q)
    arm_q, ik_success = arm_chain.inverse_kinematics_above_ground(
        target_position=target_position, target_orientation=gripper_ori, orientation_mode='all',
        initial_position=arm_q_default, optimizer=gen.ik_optimizer, regularization_parameter=gen.ik_regularization)
    if ik_success:
        debug_gripper_pos, debug_gripper_quat = get_gripper_pos_quat_from_arm_q(arm_chain, arm_q, gen.gripper_type, has_ft_sensor=gen.has_ft_sensor[motion_type])
        if not (np.allclose(grasp.pos, debug_gripper_pos, atol=1e-4) and np.allclose(grasp.quat, debug_gripper_quat, atol=1e-4)):
            ik_success = False
    return arm_chain, arm_q, ik_success


def fit_camera(scene, meshes, angles, fov=(60, 45), pad=1.25):
    '''
    scene.set_camera(distance=None, ...) under-fits rotated views (trimesh's look_at() only
    checks the 2 diagonal AABB corners, which aren't the true extreme points once rotated), so
    for the "wide" full-scene shot we instead check all 8 box corners of every mesh ourselves and
    pick a distance that's guaranteed to fit everything.
    '''
    bounds = np.array([m.bounds for m in meshes])  # (n, 2, 3)
    lo, hi = bounds[:, 0].min(axis=0), bounds[:, 1].max(axis=0)
    corners = np.array(np.meshgrid(*zip(lo, hi))).T.reshape(-1, 3)
    center = (lo + hi) / 2
    rotation = trimesh.transformations.euler_matrix(*angles)
    corners_c = rotation[:3, :3].T.dot((corners - center).T).T
    tfov = np.tan(np.radians(fov) / 2.0)
    distance = np.max(np.abs(corners_c[:, :2]) / tfov + np.abs(corners_c[:, 2:3]))
    scene.set_camera(angles=angles, distance=distance * pad, center=center, fov=fov)


def render_scene(part_mesh, gripper_meshes_i, out_dir, prefix, ground_mesh=None, arm_meshes_i=None):
    os.makedirs(out_dir, exist_ok=True)
    part_mesh_viz = part_mesh.copy()
    part_mesh_viz.visual.face_colors = [80, 200, 80, 200]
    gripper_viz = []
    for mesh in gripper_meshes_i.values():
        m = mesh.copy()
        m.visual.face_colors = [80, 120, 220, 200]
        gripper_viz.append(m)
    arm_viz = []
    if arm_meshes_i is not None:
        for mesh in arm_meshes_i.values():
            m = mesh.copy()
            m.visual.face_colors = [230, 160, 50, 160]
            arm_viz.append(m)
    scene_meshes = [part_mesh_viz] + gripper_viz + arm_viz
    if ground_mesh is not None:
        gm = ground_mesh.copy()
        gm.visual.face_colors = [220, 60, 60, 100]
        scene_meshes.append(gm)
    scene = trimesh.Scene(scene_meshes)

    close_meshes = [part_mesh_viz] + gripper_viz
    angle_sets = {
        'iso': (np.pi / 3.2, 0, np.pi / 4),
        'front': (np.pi / 2, 0, 0),
        'top': (0.01, 0, 0),
    }
    paths = []
    for view_name, angles in angle_sets.items():
        fit_camera(scene, close_meshes, angles)  # close-up: just gripper + part
        png = scene.save_image(resolution=(900, 700))
        path = os.path.join(out_dir, f'{prefix}_{view_name}.png')
        with open(path, 'wb') as fp:
            fp.write(png)
        paths.append(path)
    if arm_meshes_i is not None:
        fit_camera(scene, scene_meshes, (np.pi / 3.2, 0, np.pi / 4))  # wide: everything incl. arm
        png = scene.save_image(resolution=(900, 700))
        path = os.path.join(out_dir, f'{prefix}_wide.png')
        with open(path, 'wb') as fp:
            fp.write(png)
        paths.append(path)
    return paths


def debug_grasp(assembly_dir, log_dir, arm, gripper, part_id, failure_mode, out_dir, ft_sensor='none',
                 reduced_limit=0.1, seed=0, motion_type='move', show_arm=True):
    asset_folder = os.path.join(project_base_dir, 'assets')

    with open(os.path.join(log_dir, 'precedence.pkl'), 'rb') as fp:
        G_preced = pickle.load(fp)

    has_ft_sensor = {'move': ft_sensor in ('all', 'move'), 'hold': ft_sensor in ('all', 'hold')}
    gen = GraspArmGenerator(asset_folder, assembly_dir, G_preced, gripper, arm, has_ft_sensor,
                             seed=seed, n_surface_pt=200, n_angle=10, antipodal_thres=0.95,
                             ik_optimizer='least_squares', ik_regularization=1.0, offset_delta=0.0, reduced_limit=reduced_limit)

    part_mesh, grasps_cand = build_candidates(gen, part_id)
    print(f'part {part_id}: {len(grasps_cand)} candidate grasps generated')

    n_match = 0
    example = None
    example_result = None  # for 'success': the actual check_grasp_feasible() return value, so the
                            # rendered arm pose is the *real* one, not a fresh (possibly different,
                            # since multi-restart least_squares IK draws from global np.random)
                            # re-solve of the same IK problem.
    for grasp in grasps_cand:
        if failure_mode == 'success':
            result = gen.check_grasp_feasible(grasp, grasp.part_id, verbose=False)
            matched = result is not None
        else:
            result = None
            matched = FAILURE_MODES[failure_mode](gen, grasp)
        if matched:
            n_match += 1
            if example is None:
                example, example_result = grasp, result
    print(f'{n_match}/{len(grasps_cand)} candidates match "{failure_mode}" ({100 * n_match / max(len(grasps_cand), 1):.1f}%)')

    if example is None:
        print(f'no candidate found matching "{failure_mode}"')
        return

    gt = get_gripper_meshes_transforms(gen.gripper_type, gen.gripper_meshes, example.pos, example.quat, np.eye(4), min(example.open_ratio + 0.05, 1.0))
    gripper_meshes_i = {name: gen.gripper_meshes[name].copy().apply_transform(t) for name, t in gt.items()}

    arm_meshes_i = None
    if show_arm:
        if failure_mode == 'success':
            # reuse the exact arm_q the real check found, for whichever motion type actually
            # succeeded (a 'success' match only requires move OR hold to be non-None)
            grasp_result = example_result['move'][0] if example_result['move'] is not None else example_result['hold']
            resolved_motion_type = 'move' if example_result['move'] is not None else 'hold'
            if motion_type != resolved_motion_type:
                print(f'  (--motion-type {motion_type} did not succeed for this candidate; showing {resolved_motion_type} instead, which did)')
            arm_chain = get_arm_chain(arm, resolved_motion_type, base_pos=grasp_result.arm_pos, base_euler=grasp_result.arm_euler, reduced_limit=reduced_limit)
            arm_q, ik_success = grasp_result.arm_q, True
        else:
            arm_chain, arm_q, ik_success = compute_arm_pose(gen, example, motion_type)
        print(f'arm pose: {"success" if ik_success else "FAILED"} (base_pos={arm_chain.base_pos}, base_euler={arm_chain.base_euler})')
        if ik_success:
            arm_meshes_visual = load_arm_meshes(arm, asset_folder, visual=True)
            arm_meshes_i = transform_arm_meshes(arm_meshes_visual, arm_chain, arm_q)
        else:
            print('  (rendering without arm since IK failed; pass --no-arm to skip the IK attempt entirely)')

    ground_mesh = trimesh.creation.box((40, 40, 0.4))
    paths = render_scene(part_mesh, gripper_meshes_i, out_dir, f'{arm}_{gripper}_part{part_id}_{failure_mode}',
                          ground_mesh=ground_mesh, arm_meshes_i=arm_meshes_i)
    print('wrote:')
    for path in paths:
        print(' ', path)


if __name__ == '__main__':
    from argparse import ArgumentParser

    parser = ArgumentParser(description='Render a failing (or successful) grasp candidate -- gripper, part, and (optionally) arm -- to PNG for visual debugging, without a display.')
    parser.add_argument('--assembly-dir', type=str, required=True, help='directory of assembly')
    parser.add_argument('--log-dir', type=str, required=True, help='directory with precedence.pkl (from run_preced_plan.py)')
    parser.add_argument('--arm', type=str, default='kuka')
    parser.add_argument('--gripper', type=str, default='kuka')
    parser.add_argument('--ft-sensor', type=str, default='none', choices=['none', 'all', 'move', 'hold'])
    parser.add_argument('--reduced-limit', type=float, default=0.1)
    parser.add_argument('--part-id', type=str, required=True, help='part id to sample grasp candidates for')
    parser.add_argument('--failure-mode', type=str, default='ground-collision', choices=list(FAILURE_MODES.keys()),
                         help='which check to find a matching candidate for: ground-collision, self-collision (gripper hits the part it is grasping), zero-contact (no sampled contact points), success (passes full check_grasp_feasible)')
    parser.add_argument('--motion-type', type=str, default='move', choices=['move', 'hold'], help='which arm chain (and base placement) to solve IK for')
    parser.add_argument('--no-arm', dest='show_arm', action='store_false', help='skip the arm IK + render (gripper + part only, faster)')
    parser.add_argument('--out-dir', type=str, default='/tmp/fabrica_grasp_debug', help='directory to write rendered PNGs to')
    parser.add_argument('--seed', type=int, default=0)
    args = parser.parse_args()

    debug_grasp(args.assembly_dir, args.log_dir, args.arm, args.gripper, args.part_id, args.failure_mode, args.out_dir,
                args.ft_sensor, args.reduced_limit, args.seed, args.motion_type, args.show_arm)
