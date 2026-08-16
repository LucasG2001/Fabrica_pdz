import math
import torch

from isaacgymenvs.utils import torch_jit_utils as torch_utils
import isaacgymenvs.tasks.factory.factory_control as fc


def compute_dof_pos_target(cfg_ctrl,
                           arm_dof_pos,
                           fingertip_midpoint_pos,
                           fingertip_midpoint_quat,
                           jacobian,
                           ctrl_target_fingertip_midpoint_pos,
                           ctrl_target_fingertip_midpoint_quat,
                           device):
    """Compute Kuka DOF position target to move fingertips towards target pose."""

    ctrl_target_dof_pos = torch.zeros((cfg_ctrl['num_envs'], 9), device=device)

    pos_error, axis_angle_error = fc.get_pose_error(
        fingertip_midpoint_pos=fingertip_midpoint_pos,
        fingertip_midpoint_quat=fingertip_midpoint_quat,
        ctrl_target_fingertip_midpoint_pos=ctrl_target_fingertip_midpoint_pos,
        ctrl_target_fingertip_midpoint_quat=ctrl_target_fingertip_midpoint_quat,
        jacobian_type=cfg_ctrl['jacobian_type'],
        rot_error_type='axis_angle')

    delta_fingertip_pose = torch.cat((pos_error, axis_angle_error), dim=1)
    delta_arm_dof_pos = fc._get_delta_dof_pos(delta_pose=delta_fingertip_pose,
                                           ik_method=cfg_ctrl['ik_method'],
                                           jacobian=jacobian,
                                           device=device)

    ctrl_target_dof_pos[:, 0:7] = arm_dof_pos + delta_arm_dof_pos

    return ctrl_target_dof_pos


def compute_dof_torque(cfg_ctrl,
                       dof_vel,
                       fingertip_midpoint_pos,
                       fingertip_midpoint_quat,
                       fingertip_midpoint_linvel,
                       fingertip_midpoint_angvel,
                       left_finger_force,
                       right_finger_force,
                       jacobian,
                       arm_mass_matrix,
                       ctrl_target_fingertip_midpoint_pos,
                       ctrl_target_fingertip_midpoint_quat,
                       ctrl_target_fingertip_contact_wrench,
                       device):
    """Compute Kuka DOF torque to move fingertips towards target pose."""
    # References:
    # 1) https://ethz.ch/content/dam/ethz/special-interest/mavt/robotics-n-intelligent-systems/rsl-dam/documents/RobotDynamics2018/RD_HS2018script.pdf
    # 2) Modern Robotics

    dof_torque = torch.zeros((cfg_ctrl['num_envs'], 7), device=device)

    if cfg_ctrl['gain_space'] == 'joint':
        pos_error, axis_angle_error = fc.get_pose_error(
            fingertip_midpoint_pos=fingertip_midpoint_pos,
            fingertip_midpoint_quat=fingertip_midpoint_quat,
            ctrl_target_fingertip_midpoint_pos=ctrl_target_fingertip_midpoint_pos,
            ctrl_target_fingertip_midpoint_quat=ctrl_target_fingertip_midpoint_quat,
            jacobian_type=cfg_ctrl['jacobian_type'],
            rot_error_type='axis_angle')
        delta_fingertip_pose = torch.cat((pos_error, axis_angle_error), dim=1)

        # Set tau = k_p * joint_pos_error - k_d * joint_vel_error (ETH eq. 3.72)
        delta_arm_dof_pos = fc._get_delta_dof_pos(delta_pose=delta_fingertip_pose,
                                               ik_method=cfg_ctrl['ik_method'],
                                               jacobian=jacobian,
                                               device=device)
        dof_torque = cfg_ctrl['joint_prop_gains'] * delta_arm_dof_pos + \
                             cfg_ctrl['joint_deriv_gains'] * (0.0 - dof_vel[:])

        if cfg_ctrl['do_inertial_comp']:
            # Set tau = M * tau, where M is the joint-space mass matrix
            arm_mass_matrix_joint = arm_mass_matrix
            dof_torque = (arm_mass_matrix_joint @ dof_torque.unsqueeze(-1)).squeeze(-1)

    elif cfg_ctrl['gain_space'] == 'task':
        task_wrench = torch.zeros((cfg_ctrl['num_envs'], 6), device=device)

        if cfg_ctrl['do_motion_ctrl']:
            pos_error, axis_angle_error = fc.get_pose_error(
                fingertip_midpoint_pos=fingertip_midpoint_pos,
                fingertip_midpoint_quat=fingertip_midpoint_quat,
                ctrl_target_fingertip_midpoint_pos=ctrl_target_fingertip_midpoint_pos,
                ctrl_target_fingertip_midpoint_quat=ctrl_target_fingertip_midpoint_quat,
                jacobian_type=cfg_ctrl['jacobian_type'],
                rot_error_type='axis_angle')
            delta_fingertip_pose = torch.cat((pos_error, axis_angle_error), dim=1)

            # Set tau = k_p * task_pos_error - k_d * task_vel_error (building towards eq. 3.96-3.98)
            task_wrench_motion = fc._apply_task_space_gains(delta_fingertip_pose=delta_fingertip_pose,
                                                         fingertip_midpoint_linvel=fingertip_midpoint_linvel,
                                                         fingertip_midpoint_angvel=fingertip_midpoint_angvel,
                                                         task_prop_gains=cfg_ctrl['task_prop_gains'],
                                                         task_deriv_gains=cfg_ctrl['task_deriv_gains'])

            if cfg_ctrl['do_inertial_comp']:
                # Set tau = Lambda * tau, where Lambda is the task-space mass matrix
                jacobian_T = torch.transpose(jacobian, dim0=1, dim1=2)
                arm_mass_matrix_task = torch.inverse(jacobian @ torch.inverse(arm_mass_matrix) @ jacobian_T)  # ETH eq. 3.86; geometric Jacobian is assumed
                task_wrench_motion = (arm_mass_matrix_task @ task_wrench_motion.unsqueeze(-1)).squeeze(-1)

            task_wrench = task_wrench + torch.tensor(cfg_ctrl['motion_ctrl_axes'], device=device).unsqueeze(0) * task_wrench_motion

        if cfg_ctrl['do_force_ctrl']:
            # Set tau = tau + F_t, where F_t is the target contact wrench
            task_wrench_force = torch.zeros((cfg_ctrl['num_envs'], 6), device=device)
            task_wrench_force = task_wrench_force + ctrl_target_fingertip_contact_wrench  # open-loop force control (building towards ETH eq. 3.96-3.98)

            if cfg_ctrl['force_ctrl_method'] == 'closed':
                force_error, torque_error = fc._get_wrench_error(
                    left_finger_force=left_finger_force,
                    right_finger_force=right_finger_force,
                    ctrl_target_fingertip_contact_wrench=ctrl_target_fingertip_contact_wrench,
                    num_envs=cfg_ctrl['num_envs'],
                    device=device)

                # Set tau = tau + k_p * contact_wrench_error
                task_wrench_force = task_wrench_force + cfg_ctrl['wrench_prop_gains'] * torch.cat(
                    (force_error, torque_error), dim=1)  # part of Modern Robotics eq. 11.61

            task_wrench = task_wrench + torch.tensor(cfg_ctrl['force_ctrl_axes'], device=device).unsqueeze(
                0) * task_wrench_force

        # Set tau = J^T * tau, i.e., map tau into joint space as desired
        jacobian_T = torch.transpose(jacobian, dim0=1, dim1=2)
        dof_torque = (jacobian_T @ task_wrench.unsqueeze(-1)).squeeze(-1)

    dof_torque = torch.clamp(dof_torque, min=-100.0, max=100.0)

    return dof_torque
