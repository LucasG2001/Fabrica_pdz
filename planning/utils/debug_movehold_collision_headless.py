import os
import sys

project_base_dir = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..'))
sys.path.append(project_base_dir)
os.environ['OMP_NUM_THREADS'] = '1'

import pickle
import numpy as np
import trimesh

from planning.run_grasp_arm_gen import GraspArmGenerator
from planning.robot.geometry import get_gripper_meshes_transforms, load_arm_meshes, get_arm_meshes_transforms, get_gripper_finger_names
from planning.robot.util_arm import get_arm_chain
from planning.config import RETRACT_OPEN_RATIO
from planning.utils.debug_grasp_headless import fit_camera


def load_grasps(log_dir):
    '''
    grasps.pkl['grasps'][part_id]['move'] is a list of candidates, each candidate itself a list of
    n_timestep Grasp objects (waypoints from grasp pose up to a retracted hover pose -- the same
    waypoints check_grasp_id_pair_feasible_batch checks for collision one at a time, not just the
    final pose). ['hold'] candidates are single static Grasp objects (the holding arm doesn't move).
    '''
    with open(os.path.join(log_dir, 'grasps.pkl'), 'rb') as fp:
        data = pickle.load(fp)
    return data


def find_colliding_pair(gen, move_candidates, hold_candidates):
    '''
    Replicates GraspArmGenerator.check_grasp_id_pair_feasible_batch's move-vs-hold collision check
    (buffered move gripper+arm meshes, at each waypoint, against unbuffered hold gripper+arm meshes)
    for individual (move, hold) candidate pairs, using fresh local collision managers so gen's own
    persistent state is untouched. Returns the first colliding match found, along with which waypoint
    index and which mesh names collided, or None if no collision is found among the given candidates.
    '''
    for move_waypoints in move_candidates:
        for hold_grasp in hold_candidates:
            col_manager_hold = trimesh.collision.CollisionManager()
            gt_hold = get_gripper_meshes_transforms(gen.gripper_type, gen.gripper_meshes, hold_grasp.pos, hold_grasp.quat, np.eye(4), hold_grasp.open_ratio)
            gt_hold_open = get_gripper_meshes_transforms(gen.gripper_type, gen.gripper_meshes, hold_grasp.pos, hold_grasp.quat, np.eye(4), min(hold_grasp.open_ratio + RETRACT_OPEN_RATIO, 1.0))
            at_hold = get_arm_meshes_transforms(gen.arm_meshes, gen.arm_chains['hold'], hold_grasp.arm_q)
            for name, t in gt_hold.items(): col_manager_hold.add_object(name, gen.gripper_meshes[name], transform=t)
            for name, t in gt_hold_open.items(): col_manager_hold.add_object(name + '_open', gen.gripper_meshes[name], transform=t)
            for name, t in at_hold.items(): col_manager_hold.add_object(name, gen.arm_meshes[name], transform=t)

            for i, grasp_move_i in enumerate(move_waypoints):
                col_manager_move = trimesh.collision.CollisionManager()
                gt_move = get_gripper_meshes_transforms(gen.gripper_type, gen.gripper_meshes_buffered, grasp_move_i.pos, grasp_move_i.quat, np.eye(4), grasp_move_i.open_ratio)
                gt_move_open = get_gripper_meshes_transforms(gen.gripper_type, gen.gripper_meshes_buffered, grasp_move_i.pos, grasp_move_i.quat, np.eye(4), min(grasp_move_i.open_ratio + RETRACT_OPEN_RATIO, 1.0))
                at_move = get_arm_meshes_transforms(gen.arm_meshes_buffered, gen.arm_chains['move'], grasp_move_i.arm_q)
                for name, t in gt_move.items(): col_manager_move.add_object(name, gen.gripper_meshes_buffered[name], transform=t)
                for name, t in gt_move_open.items(): col_manager_move.add_object(name + '_open', gen.gripper_meshes_buffered[name], transform=t)
                for name, t in at_move.items(): col_manager_move.add_object(name, gen.arm_meshes_buffered[name], transform=t)

                in_collision, contact_data = col_manager_move.in_collision_other(col_manager_hold, return_data=True)
                if in_collision:
                    contact_names = sorted(set(n for cdata in contact_data for n in cdata.names))
                    return {
                        'move_grasp': grasp_move_i, 'move_waypoint_index': i, 'hold_grasp': hold_grasp,
                        'contact_names': contact_names,
                    }
    return None


def render_movehold_collision(gen, match, part_move, part_hold, out_dir, prefix, arm_type, asset_folder, reduced_limit):
    os.makedirs(out_dir, exist_ok=True)
    grasp_move, grasp_hold = match['move_grasp'], match['hold_grasp']

    part_move_mesh = gen.part_meshes[part_move].copy().apply_transform(gen.part_final_transforms[part_move])
    part_move_mesh.visual.face_colors = [80, 200, 80, 200]
    part_hold_mesh = gen.part_meshes[part_hold].copy().apply_transform(gen.part_final_transforms[part_hold])
    part_hold_mesh.visual.face_colors = [80, 200, 80, 200]

    arm_meshes_visual = load_arm_meshes(arm_type, asset_folder, visual=True)

    move_chain = get_arm_chain(arm_type, 'move', base_pos=grasp_move.arm_pos, base_euler=grasp_move.arm_euler, reduced_limit=reduced_limit)
    gt_move = get_gripper_meshes_transforms(gen.gripper_type, gen.gripper_meshes_visual, grasp_move.pos, grasp_move.quat, np.eye(4), grasp_move.open_ratio)
    gripper_move_viz = {name: gen.gripper_meshes_visual[name].copy().apply_transform(t) for name, t in gt_move.items()}
    at_move = get_arm_meshes_transforms(arm_meshes_visual, move_chain, grasp_move.arm_q)
    arm_move_viz = {name: arm_meshes_visual[name].copy().apply_transform(t) for name, t in at_move.items()}
    for m in list(gripper_move_viz.values()) + list(arm_move_viz.values()):
        # arm link meshes carry a baked-in texture/material (KUKA decal), which silently overrides a
        # plain .visual.face_colors assignment -- force a fresh ColorVisuals so the move/hold tint
        # actually shows up on the arm bodies too, not just the (untextured) gripper meshes
        m.visual = trimesh.visual.ColorVisuals(m, face_colors=[80, 120, 220, 210])  # blue: move arm/gripper (the one being commanded here)

    hold_chain = get_arm_chain(arm_type, 'hold', base_pos=grasp_hold.arm_pos, base_euler=grasp_hold.arm_euler, reduced_limit=reduced_limit)
    gt_hold = get_gripper_meshes_transforms(gen.gripper_type, gen.gripper_meshes_visual, grasp_hold.pos, grasp_hold.quat, np.eye(4), grasp_hold.open_ratio)
    gripper_hold_viz = {name: gen.gripper_meshes_visual[name].copy().apply_transform(t) for name, t in gt_hold.items()}
    at_hold = get_arm_meshes_transforms(arm_meshes_visual, hold_chain, grasp_hold.arm_q)
    arm_hold_viz = {name: arm_meshes_visual[name].copy().apply_transform(t) for name, t in at_hold.items()}
    for m in list(gripper_hold_viz.values()) + list(arm_hold_viz.values()):
        m.visual = trimesh.visual.ColorVisuals(m, face_colors=[230, 160, 50, 210])  # orange: hold arm/gripper (stationary)

    ground_mesh = trimesh.creation.box((40, 40, 0.4))
    ground_mesh.visual.face_colors = [220, 60, 60, 100]

    scene_meshes = [part_move_mesh, part_hold_mesh] + list(gripper_move_viz.values()) + list(arm_move_viz.values()) \
        + list(gripper_hold_viz.values()) + list(arm_hold_viz.values()) + [ground_mesh]
    scene = trimesh.Scene(scene_meshes)

    angle_sets = {
        'iso': (np.pi / 3.2, 0, np.pi / 4),
        'front': (np.pi / 2, 0, 0),
        'top': (0.01, 0, 0),
        'side': (np.pi / 2, 0, np.pi / 2),
    }
    paths = []
    for view_name, angles in angle_sets.items():
        fit_camera(scene, scene_meshes, angles)
        png = scene.save_image(resolution=(1100, 850))
        path = os.path.join(out_dir, f'{prefix}_{view_name}.png')
        with open(path, 'wb') as fp:
            fp.write(png)
        paths.append(path)
    return paths


def debug_movehold_collision(assembly_dir, log_dir, part_move, part_hold, arm='kuka', gripper='kuka', ft_sensor='none',
                              reduced_limit=0.1, out_dir=None, max_pairs_checked=None):
    project_base_dir_local = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..'))
    asset_folder = os.path.join(project_base_dir_local, 'assets')

    data = load_grasps(log_dir)
    if data['arm'] != arm or data['gripper'] != gripper:
        print(f'warning: grasps.pkl was generated with arm={data["arm"]} gripper={data["gripper"]}, but --arm={arm} --gripper={gripper} was passed')

    with open(os.path.join(log_dir, 'precedence.pkl'), 'rb') as fp:
        G_preced = pickle.load(fp)

    has_ft_sensor = {'move': ft_sensor in ('all', 'move'), 'hold': ft_sensor in ('all', 'hold')}
    gen = GraspArmGenerator(asset_folder, assembly_dir, G_preced, gripper, arm, has_ft_sensor,
                             n_surface_pt=1, n_angle=1, reduced_limit=reduced_limit)

    move_candidates = data['grasps'][str(part_move)]['move']
    hold_candidates = data['grasps'][str(part_hold)]['hold']
    if max_pairs_checked is not None:
        move_candidates = move_candidates[:max_pairs_checked]
        hold_candidates = hold_candidates[:max_pairs_checked]
    print(f'checking {len(move_candidates)} move candidates (part {part_move}) x {len(hold_candidates)} hold candidates (part {part_hold})...')

    match = find_colliding_pair(gen, move_candidates, hold_candidates)
    if match is None:
        print(f'no move-hold collision found among the checked candidates for part pair ({part_move}, {part_hold})')
        return

    print(f'found collision: move grasp_id={match["move_grasp"].grasp_id} (waypoint {match["move_waypoint_index"]}) '
          f'vs hold grasp_id={match["hold_grasp"].grasp_id}')
    print(f'colliding mesh names: {match["contact_names"]}')

    if out_dir is None:
        out_dir = os.path.join(log_dir, 'grasp_debug')
    prefix = f'{arm}_{gripper}_movehold_part{part_move}-{part_hold}'
    paths = render_movehold_collision(gen, match, part_move, part_hold, out_dir, prefix, arm, asset_folder, reduced_limit)
    print('wrote:')
    for path in paths:
        print(' ', path)


if __name__ == '__main__':
    from argparse import ArgumentParser

    parser = ArgumentParser(description='Find one concrete move-hold collision from an existing grasps.pkl and render '
                                         'the exact commanded poses (both arms + grippers + both parts) to PNG for visual debugging, without a display.')
    parser.add_argument('--assembly-dir', type=str, required=True, help='directory of assembly')
    parser.add_argument('--log-dir', type=str, required=True, help='directory with grasps.pkl and precedence.pkl')
    parser.add_argument('--part-move', type=str, required=True, help='part id being moved/grasped')
    parser.add_argument('--part-hold', type=str, required=True, help='part id being held stationary')
    parser.add_argument('--arm', type=str, default='kuka')
    parser.add_argument('--gripper', type=str, default='kuka')
    parser.add_argument('--ft-sensor', type=str, default='none', choices=['none', 'all', 'move', 'hold'])
    parser.add_argument('--reduced-limit', type=float, default=0.1)
    parser.add_argument('--out-dir', type=str, default=None, help='directory to write rendered PNGs to (default: <log-dir>/grasp_debug)')
    parser.add_argument('--max-pairs-checked', type=int, default=None, help='cap on candidates scanned per side, for a quick check (default: all)')
    args = parser.parse_args()

    debug_movehold_collision(args.assembly_dir, args.log_dir, args.part_move, args.part_hold, args.arm, args.gripper,
                              args.ft_sensor, args.reduced_limit, args.out_dir, args.max_pairs_checked)
