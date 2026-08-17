import os
import sys

project_base_dir = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..'))
sys.path.append(project_base_dir)
os.environ['OMP_NUM_THREADS'] = '1'

import numpy as np
import trimesh

from planning.robot.geometry import load_arm_meshes, transform_arm_meshes, load_gripper_meshes, get_gripper_meshes_transforms
from planning.robot.util_arm import get_arm_chain, get_gripper_pos_quat_from_arm_q
from planning.utils.debug_grasp_headless import fit_camera

# Gripper type matching each supported arm (None = no gripper mesh available for that arm).
ARM_GRIPPER_TYPE = {'kuka': 'kuka', 'panda': 'panda', 'xarm7': None, 'ur5e': None}

ARM_COLOR = {
    'kuka': [230, 160, 50, 200],   # orange
    'panda': [80, 140, 230, 200],  # blue
}


def build_robot_meshes(arm_type, asset_folder, base_pos, base_euler, open_ratio, reduced_limit, show_gripper):
    arm_chain = get_arm_chain(arm_type, base_pos=base_pos, base_euler=base_euler, reduced_limit=reduced_limit)
    arm_q = arm_chain.active_to_full(arm_chain.rest_q)

    arm_meshes_visual = load_arm_meshes(arm_type, asset_folder, visual=True)
    arm_meshes_i = transform_arm_meshes(arm_meshes_visual, arm_chain, arm_q)

    gripper_meshes_i = {}
    gripper_type = ARM_GRIPPER_TYPE.get(arm_type)
    if show_gripper and gripper_type is not None:
        gripper_meshes = load_gripper_meshes(gripper_type, asset_folder, visual=True)
        gripper_pos, gripper_quat = get_gripper_pos_quat_from_arm_q(arm_chain, arm_q, gripper_type)
        gt = get_gripper_meshes_transforms(gripper_type, gripper_meshes, gripper_pos, gripper_quat, np.eye(4), open_ratio)
        gripper_meshes_i = {name: gripper_meshes[name].copy().apply_transform(t) for name, t in gt.items()}

    return arm_meshes_i, gripper_meshes_i


def render_dual_robot(arm1, arm2, out_dir, spacing=150.0, open_ratio=0.5, reduced_limit=0.1, show_gripper=True):
    asset_folder = os.path.join(project_base_dir, 'assets')
    os.makedirs(out_dir, exist_ok=True)

    base_euler = [0, 0, 0]
    robots = [
        (arm1, np.array([0.0, -spacing / 2, 0.0])),
        (arm2, np.array([0.0, spacing / 2, 0.0])),
    ]

    scene_meshes = []
    for arm_type, base_pos in robots:
        arm_meshes_i, gripper_meshes_i = build_robot_meshes(
            arm_type, asset_folder, base_pos, base_euler, open_ratio, reduced_limit, show_gripper)
        color = ARM_COLOR.get(arm_type, [150, 150, 150, 200])
        for mesh in list(arm_meshes_i.values()) + list(gripper_meshes_i.values()):
            m = mesh.copy()
            m.visual.face_colors = color
            scene_meshes.append(m)

    ground_mesh = trimesh.creation.box((300, 300, 0.4))
    ground_mesh.visual.face_colors = [200, 200, 200, 100]
    scene_meshes.append(ground_mesh)

    scene = trimesh.Scene(scene_meshes)

    angle_sets = {
        'iso': (np.pi / 3.2, 0, np.pi / 4),
        'front': (np.pi / 2, 0, 0),
        'side': (np.pi / 2, 0, np.pi / 2),
        'top': (0.01, 0, 0),
    }
    paths = []
    for view_name, angles in angle_sets.items():
        fit_camera(scene, scene_meshes, angles)
        png = scene.save_image(resolution=(1200, 900))
        path = os.path.join(out_dir, f'{arm1}_{arm2}_{view_name}.png')
        with open(path, 'wb') as fp:
            fp.write(png)
        paths.append(path)
    return paths


if __name__ == '__main__':
    from argparse import ArgumentParser

    parser = ArgumentParser(description='Render two robot arms (e.g. KUKA and Franka/Panda) side by side, at rest pose, without a display.')
    parser.add_argument('--arm1', type=str, default='kuka', choices=list(ARM_GRIPPER_TYPE.keys()))
    parser.add_argument('--arm2', type=str, default='panda', choices=list(ARM_GRIPPER_TYPE.keys()))
    parser.add_argument('--spacing', type=float, default=150.0, help='distance (cm) between the two robot bases')
    parser.add_argument('--open-ratio', type=float, default=0.5, help='gripper open ratio (0=closed, 1=open) shown for each arm')
    parser.add_argument('--reduced-limit', type=float, default=0.1)
    parser.add_argument('--no-gripper', dest='show_gripper', action='store_false', help='render arm links only, skip the end-effector gripper')
    parser.add_argument('--out-dir', type=str, default='/tmp/fabrica_dual_robot', help='directory to write rendered PNGs to')
    args = parser.parse_args()

    paths = render_dual_robot(args.arm1, args.arm2, args.out_dir, args.spacing, args.open_ratio, args.reduced_limit, args.show_gripper)
    print('wrote:')
    for path in paths:
        print(' ', path)
