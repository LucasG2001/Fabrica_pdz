import os
import sys

project_base_dir = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..'))
sys.path.append(project_base_dir)

import numpy as np
from scipy.spatial.transform import Rotation as R
from .ik.chain import Chain

from assets.transform import get_transform_matrix_quat, get_pos_euler_from_transform_matrix
from planning.robot.geometry import get_ft_sensor_spec, get_gripper_basis_directions
from planning.robot.workcell import get_move_arm_pos, get_move_arm_euler, get_hold_arm_pos, get_hold_arm_euler


def get_xarm7_arm_chain(base_pos, base_euler, reduced_limit=0.0):
    chain = Chain.from_urdf_file(os.path.join(project_base_dir, 'assets/xarm7/xarm7.urdf'), base_elements=['linkbase'],
        origin_translation=np.array(base_pos), origin_orientation=np.array(base_euler), scale_translation=100, reduced_limit=reduced_limit)
    chain.rest_q = [0., -0.56582579, 0., 0.35527904, 0., 0.92109843, 0.]
    return chain


def get_panda_arm_chain(base_pos, base_euler, reduced_limit=0.0):
    chain = Chain.from_urdf_file(os.path.join(project_base_dir, 'assets/panda/panda.urdf'), base_elements=['panda_link0'],
        origin_translation=np.array(base_pos), origin_orientation=np.array(base_euler), scale_translation=100, reduced_limit=reduced_limit)
    chain.rest_q = [0, -np.pi / 4, 0, -3 * np.pi / 4, 0, np.pi / 2, np.pi / 4]
    return chain


def get_kuka_arm_chain(base_pos, base_euler, reduced_limit=0.0):
    chain = Chain.from_urdf_file(os.path.join(project_base_dir, 'assets/kuka/kuka.urdf'), base_elements=['kuka_link0'],
        origin_translation=np.array(base_pos), origin_orientation=np.array(base_euler), scale_translation=100, reduced_limit=reduced_limit)
    # UPDATED (2026-08-24): was the Panda-derived rest pose below (kept for the RL side's
    # historical context -- see git history / learning/isaacgymenvs/cfg/task/FabricaBase.yaml's
    # kuka_rest_dof_pos, which is a separately-maintained YAML this change does NOT touch).
    #   [0, -np.pi / 4, 0, -1.9198621771937625, 0, np.pi / 2, np.pi / 4]
    # Replaced with the reference dual-arm-kuka branch's KUKA-specific high-elbow rest pose,
    # confirmed by an exact numeric replay check: this is the rest_q that pdz plumbers_block
    # grasps were actually solved/regularized against (reconstructing a stored grasp's flange
    # quaternion from its arm_q via get_gripper_pos_quat_from_arm_q only matches exactly at this
    # rest_q -- any other value, including the old one above, produces a wrong constant rotation
    # for any gripper that goes through the general basis-alignment path in
    # get_gripper_init_matrix, i.e. everything except gripper_type=='kuka' itself, which is
    # unaffected since its own shortcut cancels rest_q out entirely). Pitch angles sum to pi so
    # the flange still points straight down at rest; A4=-1.6 stays inside the reduced bound
    # +-1.662 when reduced_limit=0.1.
    chain.rest_q = [0., 0.2, 0., -1.6, 0., np.pi - 0.2 - 1.6, 0.]
    return chain


def get_ur5e_arm_chain(base_pos, base_euler, reduced_limit=0.0):
    chain = Chain.from_urdf_file(os.path.join(project_base_dir, 'assets/ur5e/ur5e.urdf'), base_elements=['base_link'],
        origin_translation=np.array(base_pos), origin_orientation=np.array(base_euler), scale_translation=1, reduced_limit=reduced_limit)
    chain.rest_q = [0., -1.57079632679, 1.57079632679, -1.57079632679, -1.57079632679, 0.]
    chain.no_collision_links = [('base_link', 'upper_arm_link'), ('wrist_1_link', 'wrist_3_link')]
    return chain


def get_arm_chain(arm_type, motion_type=None, base_pos=None, base_euler=None, reduced_limit=0.0):

    # get base position and orientation
    if motion_type is not None:
        if motion_type == 'move':
            if base_pos is None: base_pos = get_move_arm_pos(arm_type)
            if base_euler is None: base_euler = get_move_arm_euler()
        elif motion_type == 'hold':
            if base_pos is None: base_pos = get_hold_arm_pos(arm_type)
            if base_euler is None: base_euler = get_hold_arm_euler()
        else:
            raise ValueError('Unknown motion type: {}'.format(motion_type))
    else:
        assert base_pos is not None and base_euler is not None
        
    # create kinematic chain
    if arm_type == 'xarm7':
        arm_chain = get_xarm7_arm_chain(base_pos, base_euler, reduced_limit=reduced_limit)
    elif arm_type == 'panda':
        arm_chain = get_panda_arm_chain(base_pos, base_euler, reduced_limit=reduced_limit)
    elif arm_type == 'kuka':
        arm_chain = get_kuka_arm_chain(base_pos, base_euler, reduced_limit=reduced_limit)
    elif arm_type == 'ur5e':
        arm_chain = get_ur5e_arm_chain(base_pos, base_euler, reduced_limit=reduced_limit)
    else:
        raise ValueError('Unknown arm type: {}'.format(arm_type))
    arm_chain.arm_type = arm_type
    arm_chain.base_pos = base_pos
    arm_chain.base_euler = base_euler

    # set bounds for the first link to avoid unintuitive motion
    first_link = arm_chain.get_active_link(0)
    if motion_type is None:
        pass
    elif motion_type == 'move':
        first_link.bounds = (first_link.bounds[0], min(first_link.bounds[1], 0.5))
    elif motion_type == 'hold':
        first_link.bounds = (max(first_link.bounds[0], -0.5), first_link.bounds[1])
    else:
        raise ValueError('Unknown motion type: {}'.format(motion_type))

    # clamp rest_q into the (possibly reduced_limit-shrunk) joint bounds, so it always
    # remains a feasible IK initial guess regardless of reduced_limit or arm_type
    active_bounds = arm_chain.get_active_link_bounds()
    arm_chain.rest_q = [float(np.clip(q, lo, hi)) for q, (lo, hi) in zip(arm_chain.rest_q, active_bounds)]

    return arm_chain
    

def get_ft_pos_from_gripper_pos_quat(gripper_type, gripper_pos, gripper_quat):
    base_basis_direction, _ = get_gripper_basis_directions(gripper_type)
    ft_spec = get_ft_sensor_spec()
    gripper_rot = R.from_quat(gripper_quat[[1, 2, 3, 0]])
    ft_pos = gripper_pos + gripper_rot.apply(base_basis_direction) * ft_spec['height']
    return ft_pos


def get_gripper_init_matrix(arm_chain, gripper_type, ef_init_matrix=None):
    '''
    Constant local rotation between the arm chain's tip frame and the "gripper frame" convention
    (get_gripper_basis_directions), assuming the gripper is rigidly bolted to the tip.
    '''
    if ef_init_matrix is None:
        ef_init_matrix = arm_chain.forward_kinematics_active(arm_chain.rest_q)[:3, :3]
    if gripper_type == 'kuka':
        # kuka.urdf's chain tip (kuka_link8) already *is* the gripper mount frame
        # (gripper_base_link), including the real 180deg mount twist -- unlike Panda's chain
        # (which omits panda_hand_joint's -45deg twist), there's no missing mesh-mount rotation
        # left to reconstruct here. The heuristic below (assuming the chain tip points along a
        # fixed world direction at rest_q) is only valid if rest_q happens to put the true tip at
        # exactly that orientation -- true for Panda's candle-pose rest_q, but KUKA's rest_q
        # (adapted from Panda's by clipping one joint, not re-derived for KUKA's own kinematics)
        # does not satisfy it, which produced a real ~25-45deg constant misalignment between the
        # rendered/IK-targeted gripper and the true flange orientation. Skip the heuristic:
        # gripper frame == chain tip frame exactly, by construction of kuka.urdf's kuka_joint8.
        #
        # This shortcut is specific to the KUKA Y-gripper's own mount convention (kuka_joint8's
        # baked-in 180deg twist matches the Y-gripper's local mesh frame exactly). It must NOT
        # be extended to 'pdz': the real pdz gripper mounts with a different relationship (its
        # CAD closing axis is local +X, not the Y-gripper's +Y -- see get_pdz_basis_directions),
        # so pdz needs to fall through to the general basis-direction heuristic below instead.
        return ef_init_matrix
    base_init_direction, l2r_init_direction = [0, 0, 1], R.from_euler('xyz', arm_chain.links[0].origin_orientation).apply([0, -1, 0])
    return R.align_vectors([base_init_direction, l2r_init_direction], [*get_gripper_basis_directions(gripper_type)])[0].as_matrix()


def get_gripper_pos_quat_from_arm_q(arm_chain, arm_q, gripper_type, has_ft_sensor=False):

    ef_target_matrix = arm_chain.forward_kinematics(arm_q)
    ef_init_matrix = arm_chain.forward_kinematics_active(arm_chain.rest_q)[:3, :3]
    base_basis_direction, _ = get_gripper_basis_directions(gripper_type)
    gripper_init_matrix = get_gripper_init_matrix(arm_chain, gripper_type, ef_init_matrix)
    gripper_target_matrix = ef_target_matrix[:3, :3] @ ef_init_matrix.T @ gripper_init_matrix
    gripper_pos, gripper_quat = ef_target_matrix[:3, 3], R.from_matrix(gripper_target_matrix).as_quat()[[3, 0, 1, 2]]

    if has_ft_sensor:
        ft_spec = get_ft_sensor_spec()
        gripper_pos -= R.from_matrix(gripper_target_matrix).apply(base_basis_direction) * ft_spec['height']
    
    return gripper_pos, gripper_quat


def get_gripper_qm_from_arm_q(arm_chain, arm_q, gripper_type, has_ft_sensor=False):

    gripper_pos, gripper_quat = get_gripper_pos_quat_from_arm_q(arm_chain, arm_q, gripper_type, has_ft_sensor=has_ft_sensor)
    gripper_matrix = get_transform_matrix_quat(gripper_pos, gripper_quat)
    gripper_qm = get_pos_euler_from_transform_matrix(gripper_matrix)

    return gripper_qm


def get_gripper_path_from_arm_path(arm_chain, arm_path, gripper_type, has_ft_sensor=False):
    
    gripper_path = []
    for arm_q in arm_path:
        gripper_path.append(get_gripper_qm_from_arm_q(arm_chain, arm_q, gripper_type, has_ft_sensor=has_ft_sensor))

    return gripper_path


def get_gripper_part_qm_from_arm_q(arm_chain, arm_q, gripper_type, part_transform, has_ft_sensor=False):

    gripper_pos, gripper_quat = get_gripper_pos_quat_from_arm_q(arm_chain, arm_q, gripper_type, has_ft_sensor=has_ft_sensor)
    gripper_matrix = get_transform_matrix_quat(gripper_pos, gripper_quat)
    gripper_qm = get_pos_euler_from_transform_matrix(gripper_matrix)

    part_matrix = gripper_matrix @ part_transform
    part_qm = get_pos_euler_from_transform_matrix(part_matrix)

    return gripper_qm, part_qm


def get_gripper_part_path_from_arm_path(arm_chain, arm_path, gripper_type, part_transform, has_ft_sensor=False):
    
    gripper_path, part_path = [], []
    for arm_q in arm_path:
        gripper_qm, part_qm = get_gripper_part_qm_from_arm_q(arm_chain, arm_q, gripper_type, part_transform, has_ft_sensor=has_ft_sensor)
        gripper_path.append(gripper_qm)
        part_path.append(part_qm)

    return gripper_path, part_path


def get_ik_target_orientation(arm_chain, gripper_type, gripper_quat):
    '''
    Computes the target orientation for the end effector given the gripper orientation
    '''
    ef_init_matrix = arm_chain.forward_kinematics_active(arm_chain.rest_q)[:3, :3] # end effector initial rotation at rest pose
    gripper_init_matrix = get_gripper_init_matrix(arm_chain, gripper_type, ef_init_matrix) # gripper initial rotation

    gripper_target_matrix = R.from_quat(gripper_quat[[1, 2, 3, 0]]).as_matrix()
    ef_target_matrix = gripper_target_matrix @ gripper_init_matrix.T @ ef_init_matrix

    return ef_target_matrix


def inverse_kinematics_correction(arm_chain, arm_q, gripper_type, gripper_quat): # NOTE: deprecated
    '''
    Computes the inverse kinematic on the specified target with correction on the last active joint angle
    '''
    arm_q = arm_q.copy()

    ef_init_matrix = arm_chain.forward_kinematics_active(arm_chain.rest_q)[:3, :3] # end effector initial rotation at rest pose
    gripper_init_matrix = get_gripper_init_matrix(arm_chain, gripper_type, ef_init_matrix) # gripper initial rotation

    ef_target_matrix = R.from_quat(gripper_quat[[1, 2, 3, 0]]).as_matrix() @ gripper_init_matrix.T @ ef_init_matrix # end effector target rotation for given gripper state
    ef_curr_matrix = arm_chain.forward_kinematics(arm_q) # end effector current rotation from current joint angles

    correct_rotvec = R.from_matrix(ef_curr_matrix[:3, :3].T @ ef_target_matrix).as_rotvec() # rotation vector for last joint angle correction (NOTE: ideally should be 0, 0, theta)
    
    arm_q_active = arm_chain.active_from_full(arm_q)
    arm_q_active[-1] += correct_rotvec[-1]
    if arm_q_active[-1] < -np.pi:
        arm_q_active[-1] += np.pi * 2
    elif arm_q_active[-1] > np.pi:
        arm_q_active[-1] -= np.pi * 2
    arm_q = arm_chain.active_to_full(arm_q_active)

    return arm_q
    

def check_inverse_kinematics_success(arm_chain, arm_q, gripper_type, gripper_quat, eps=1e-3, verbose=False):

    arm_q = arm_q.copy()

    ef_init_matrix = arm_chain.forward_kinematics_active(arm_chain.rest_q)[:3, :3] # end effector initial rotation at rest pose
    gripper_init_matrix = get_gripper_init_matrix(arm_chain, gripper_type, ef_init_matrix) # gripper initial rotation

    ef_target_matrix = R.from_quat(gripper_quat[[1, 2, 3, 0]]).as_matrix() @ gripper_init_matrix.T @ ef_init_matrix # end effector target rotation for given gripper state
    ef_curr_matrix = arm_chain.forward_kinematics(arm_q) # end effector current rotation from current joint angles

    correct_rotvec = R.from_matrix(ef_curr_matrix[:3, :3].T @ ef_target_matrix).as_rotvec() # rotation vector for last joint angle correction (NOTE: ideally should be 0, 0, theta)
    deviation_norm = np.linalg.norm(correct_rotvec[:2])
    
    if verbose:
        print('IK rotation deviation: {:.4f}'.format(deviation_norm) + f', Success: {deviation_norm < eps}')

    return deviation_norm < eps


def get_arm_path_from_gripper_path(gripper_path, gripper_type, arm_chain, arm_q_init, has_ft_sensor=False):
    arm_path_local = []
    arm_q = arm_q_init.copy() if arm_q_init is not None else None # full
    for qm in gripper_path:
        gripper_pos = qm[:3]
        gripper_rot = R.from_euler('xyz', qm[3:])
        gripper_quat = gripper_rot.as_quat()[[3, 0, 1, 2]]
        gripper_ori = get_ik_target_orientation(arm_chain, gripper_type, gripper_quat)
        ft_pos = get_ft_pos_from_gripper_pos_quat(gripper_type, gripper_pos, gripper_quat) if has_ft_sensor else None

        arm_q, ik_success = arm_chain.inverse_kinematics(target_position=ft_pos if has_ft_sensor else gripper_pos, target_orientation=gripper_ori, orientation_mode='all', initial_position=arm_q, optimizer='L-BFGS-B')

        if not ik_success: # IK not fully checked for every step in the path during planning
            print('inverse kinematics failed')
        arm_path_local.append(arm_q)
    return arm_path_local
