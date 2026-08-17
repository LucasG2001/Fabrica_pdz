import os
import sys

project_base_dir = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..'))
sys.path.append(project_base_dir)
os.environ['OMP_NUM_THREADS'] = '1'

import numpy as np
import trimesh

from planning.robot.geometry import load_gripper_meshes, get_gripper_meshes_transforms
from planning.robot.util_arm import get_arm_chain

# Gripper type matching each supported arm (None = no gripper mesh available for that arm).
ARM_GRIPPER_TYPE = {'kuka': 'kuka', 'panda': 'panda'}


def render_gripper_closeup(arm_type, out_dir, open_ratio=0.5, reduced_limit=0.1, fov=(60, 45), pad=1.4):
    '''
    Close-up render of just the gripper (hand + fingers), at rest pose, with the camera aligned to
    the arm's own flange (chain-tip) frame rather than a fixed world-frame angle -- so the shot is
    meaningful regardless of what world-frame orientation the arm's rest pose happens to put the
    flange in. Camera is placed along the flange's local +/-X axis (vision ray, i.e. the camera's
    -Z axis in world, is exactly the flange's -X axis -- by construction this has zero component
    along the flange's local Y axis), with the flange's Z (approach) axis as image-up and the
    flange's Y (finger closing) axis as image-right. This is the view that makes a rotation error
    about the flange's Z axis (like the kuka_hand mount-angle bug) visible as an outright rotation
    of the finger pair in the image, rather than being hidden/foreshortened by an arbitrary
    world-frame camera choice.
    '''
    gripper_type = ARM_GRIPPER_TYPE[arm_type]
    asset_folder = os.path.join(project_base_dir, 'assets')
    os.makedirs(out_dir, exist_ok=True)

    arm_chain = get_arm_chain(arm_type, base_pos=[0, 0, 0], base_euler=[0, 0, 0], reduced_limit=reduced_limit)
    flange_transform_world = arm_chain.forward_kinematics_active(arm_chain.rest_q)
    flange_rot_world, flange_pos_world = flange_transform_world[:3, :3], flange_transform_world[:3, 3]

    gripper_meshes = load_gripper_meshes(gripper_type, asset_folder, visual=True)
    # gripper frame == chain tip (flange) frame exactly for both kuka and panda's chain tip
    # convention used elsewhere in this codebase (see get_gripper_init_matrix in util_arm.py)
    gripper_quat = trimesh.transformations.quaternion_from_matrix(
        np.vstack([np.hstack([flange_rot_world, [[0], [0], [0]]]), [0, 0, 0, 1]]))
    gt = get_gripper_meshes_transforms(gripper_type, gripper_meshes, flange_pos_world, gripper_quat, np.eye(4), open_ratio)
    part_colors = {
        'kuka_hand': [150, 150, 150, 220], 'kuka_leftfinger': [230, 20, 180, 220], 'kuka_rightfinger': [40, 220, 90, 220],
        'panda_hand': [150, 150, 150, 220], 'panda_leftfinger': [230, 20, 180, 220], 'panda_rightfinger': [40, 220, 90, 220],
    }
    scene_meshes = []
    for name, transform in gt.items():
        m = gripper_meshes[name].copy()
        m.apply_transform(transform)
        m.visual.face_colors = part_colors.get(name, [150, 150, 150, 220])
        scene_meshes.append(m)

    # frame axes marker: small cylinders at flange origin along each flange axis, to make the
    # frame itself legible in the render (black=X, yellow=Y/closing, purple=Z/approach) -- chosen
    # to not clash with the finger colors (pink=leftfinger, green=rightfinger, gray=hand)
    axis_len = 6.0
    axis_colors = {'x': [20, 20, 20, 255], 'y': [230, 210, 30, 255], 'z': [140, 40, 200, 255]}
    for i, key in enumerate(['x', 'y', 'z']):
        direction = flange_rot_world[:, i]
        cyl = trimesh.creation.cylinder(radius=0.25, segment=np.array([flange_pos_world, flange_pos_world + axis_len * direction]))
        cyl.visual.face_colors = axis_colors[key]
        scene_meshes.append(cyl)

    scene = trimesh.Scene(scene_meshes)

    flange_x_world, flange_y_world, flange_z_world = flange_rot_world[:, 0], flange_rot_world[:, 1], flange_rot_world[:, 2]
    # 'side' view: vision ray = -flange_X (zero Y-component in flange frame, per spec) -- image-right
    # is the closing axis (Y), image-up is the approach axis (Z).
    # 'muzzle' view: vision ray = -flange_Z (looking straight down the approach axis, also zero
    # Y-component) -- shows the X/Y plane full-on, which makes a Z-axis mount-rotation error read
    # directly as an angle in the image instead of a foreshortened 3D shape.
    camera_rotations = {}
    camera_rotations['side'] = np.eye(4)
    camera_rotations['side'][:3, 0] = flange_y_world
    camera_rotations['side'][:3, 1] = flange_z_world
    camera_rotations['side'][:3, 2] = flange_x_world
    camera_rotations['muzzle'] = np.eye(4)
    camera_rotations['muzzle'][:3, 0] = flange_x_world
    camera_rotations['muzzle'][:3, 1] = flange_y_world
    camera_rotations['muzzle'][:3, 2] = flange_z_world

    bounds = np.array([m.bounds for m in scene_meshes])
    lo, hi = bounds[:, 0].min(axis=0), bounds[:, 1].max(axis=0)
    corners = np.array(np.meshgrid(*zip(lo, hi))).T.reshape(-1, 3)
    center = (lo + hi) / 2

    paths = []
    for view_name, camera_rotation in camera_rotations.items():
        cam_transform = trimesh.scene.cameras.look_at(corners, fov=fov, rotation=camera_rotation, center=center, pad=pad)
        scene.camera_transform = cam_transform
        scene.camera.fov = fov
        png = scene.save_image(resolution=(1000, 1000))
        path = os.path.join(out_dir, f'{arm_type}_gripper_{view_name}_view.png')
        with open(path, 'wb') as fp:
            fp.write(png)
        paths.append(path)
    return paths


if __name__ == '__main__':
    from argparse import ArgumentParser

    parser = ArgumentParser(description='Close-up render of a gripper (hand + fingers) at rest pose, camera aligned to the flange frame (no Y-component along the vision ray).')
    parser.add_argument('--arm', type=str, default='kuka', choices=list(ARM_GRIPPER_TYPE.keys()))
    parser.add_argument('--open-ratio', type=float, default=0.5)
    parser.add_argument('--reduced-limit', type=float, default=0.1)
    parser.add_argument('--out-dir', type=str, default='output/gripper_closeup', help='directory to write the rendered PNG to')
    args = parser.parse_args()

    paths = render_gripper_closeup(args.arm, args.out_dir, args.open_ratio, args.reduced_limit)
    print('wrote:')
    for path in paths:
        print(' ', path)
