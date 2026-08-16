import hydra
import math
import os
import torch

from isaacgym import gymapi, gymtorch, torch_utils
import isaacgymenvs.tasks.factory.factory_control as fc
import isaacgymenvs.tasks.fabrica.factory_control_fixplug as fcfix
from isaacgymenvs.tasks.fabrica.fabrica_base import FabricaBase


class FabricaFixPlugBase(FabricaBase):

    def import_table_assets(self):

        table_options = gymapi.AssetOptions()
        table_options.flip_visual_attachments = False  # default = False
        table_options.fix_base_link = True
        table_options.thickness = 0.0  # default = 0.02
        table_options.density = 1000.0  # default = 1000.0
        table_options.armature = 0.0  # default = 0.0
        table_options.use_physx_armature = True
        table_options.linear_damping = 0.0  # default = 0.0
        table_options.max_linear_velocity = 1000.0  # default = 1000.0
        table_options.angular_damping = 0.0  # default = 0.5
        table_options.max_angular_velocity = 64.0  # default = 64.0
        table_options.disable_gravity = False
        table_options.enable_gyroscopic_forces = True
        table_options.default_dof_drive_mode = gymapi.DOF_MODE_NONE
        table_options.use_mesh_materials = False
        if self.cfg_base.mode.export_scene:
            table_options.mesh_normal_mode = gymapi.COMPUTE_PER_FACE

        table_asset = self.gym.create_box(
            self.sim,
            self.asset_info_kuka_table.table_depth,
            self.asset_info_kuka_table.table_width,
            self.cfg_base.env.table_height,
            table_options,
        )

        return table_asset

    def acquire_base_tensors(self):
        """Acquire and wrap tensors. Create views."""

        _root_state = self.gym.acquire_actor_root_state_tensor(
            self.sim
        )  # shape = (num_envs * num_actors, 13)
        _body_state = self.gym.acquire_rigid_body_state_tensor(
            self.sim
        )  # shape = (num_envs * num_bodies, 13)
        _dof_state = self.gym.acquire_dof_state_tensor(
            self.sim
        )  # shape = (num_envs * num_dofs, 2)
        _dof_force = self.gym.acquire_dof_force_tensor(
            self.sim
        )  # shape = (num_envs * num_dofs, 1)
        _contact_force = self.gym.acquire_net_contact_force_tensor(
            self.sim
        )  # shape = (num_envs * num_bodies, 3)
        _jacobian = self.gym.acquire_jacobian_tensor(
            self.sim, "kuka"
        )  # shape = (num envs, num_bodies, 6, num_dofs)
        _mass_matrix = self.gym.acquire_mass_matrix_tensor(
            self.sim, "kuka"
        )  # shape = (num_envs, num_dofs, num_dofs)

        self.root_state = gymtorch.wrap_tensor(_root_state)
        self.body_state = gymtorch.wrap_tensor(_body_state)
        self.dof_state = gymtorch.wrap_tensor(_dof_state)
        self.dof_force = gymtorch.wrap_tensor(_dof_force)
        self.contact_force = gymtorch.wrap_tensor(_contact_force)
        self.jacobian = gymtorch.wrap_tensor(_jacobian)
        self.mass_matrix = gymtorch.wrap_tensor(_mass_matrix)

        self.root_pos = self.root_state.view(self.num_envs, self.num_actors, 13)[
            ..., 0:3
        ]
        self.root_quat = self.root_state.view(self.num_envs, self.num_actors, 13)[
            ..., 3:7
        ]
        self.root_linvel = self.root_state.view(self.num_envs, self.num_actors, 13)[
            ..., 7:10
        ]
        self.root_angvel = self.root_state.view(self.num_envs, self.num_actors, 13)[
            ..., 10:13
        ]
        self.body_pos = self.body_state.view(self.num_envs, self.num_bodies, 13)[
            ..., 0:3
        ]
        self.body_quat = self.body_state.view(self.num_envs, self.num_bodies, 13)[
            ..., 3:7
        ]
        self.body_linvel = self.body_state.view(self.num_envs, self.num_bodies, 13)[
            ..., 7:10
        ]
        self.body_angvel = self.body_state.view(self.num_envs, self.num_bodies, 13)[
            ..., 10:13
        ]
        self.dof_pos = self.dof_state.view(self.num_envs, self.num_dofs, 2)[..., 0]
        self.dof_vel = self.dof_state.view(self.num_envs, self.num_dofs, 2)[..., 1]
        self.dof_force_view = self.dof_force.view(self.num_envs, self.num_dofs, 1)[
            ..., 0
        ]
        self.contact_force = self.contact_force.view(self.num_envs, self.num_bodies, 3)[
            ..., 0:3
        ]

        self.arm_dof_pos = self.dof_pos[:, 0:7]
        self.arm_dof_vel = self.dof_vel[:, 0:7]
        self.arm_mass_matrix = self.mass_matrix[
            :, 0:7, 0:7
        ]  # for Kuka arm (not gripper)

        self.robot_base_pos = self.body_pos[:, self.robot_base_body_id_env, 0:3]
        self.robot_base_quat = self.body_quat[:, self.robot_base_body_id_env, 0:4]

        self.hand_pos = self.body_pos[:, self.hand_body_id_env, 0:3]
        self.hand_quat = self.body_quat[:, self.hand_body_id_env, 0:4]
        self.hand_linvel = self.body_linvel[:, self.hand_body_id_env, 0:3]
        self.hand_angvel = self.body_angvel[:, self.hand_body_id_env, 0:3]
        self.hand_jacobian = self.jacobian[
            :, self.hand_body_id_env_actor - 1, 0:6, 0:7
        ]  # minus 1 because base is fixed

        self.left_finger_pos = self.body_pos[:, self.left_finger_body_id_env, 0:3]
        self.left_finger_quat = self.body_quat[:, self.left_finger_body_id_env, 0:4]
        self.left_finger_linvel = self.body_linvel[:, self.left_finger_body_id_env, 0:3]
        self.left_finger_angvel = self.body_angvel[:, self.left_finger_body_id_env, 0:3]
        self.left_finger_jacobian = self.jacobian[
            :, self.left_finger_body_id_env_actor - 1, 0:6, 0:7
        ]  # minus 1 because base is fixed

        self.right_finger_pos = self.body_pos[:, self.right_finger_body_id_env, 0:3]
        self.right_finger_quat = self.body_quat[:, self.right_finger_body_id_env, 0:4]
        self.right_finger_linvel = self.body_linvel[
            :, self.right_finger_body_id_env, 0:3
        ]
        self.right_finger_angvel = self.body_angvel[
            :, self.right_finger_body_id_env, 0:3
        ]
        self.right_finger_jacobian = self.jacobian[
            :, self.right_finger_body_id_env_actor - 1, 0:6, 0:7
        ]  # minus 1 because base is fixed

        self.left_finger_force = self.contact_force[
            :, self.left_finger_body_id_env, 0:3
        ]
        self.right_finger_force = self.contact_force[
            :, self.right_finger_body_id_env, 0:3
        ]

        self.fingertip_centered_pos = self.body_pos[
            :, self.fingertip_centered_body_id_env, 0:3
        ]
        self.fingertip_centered_quat = self.body_quat[
            :, self.fingertip_centered_body_id_env, 0:4
        ]
        self.fingertip_centered_linvel = self.body_linvel[
            :, self.fingertip_centered_body_id_env, 0:3
        ]
        self.fingertip_centered_angvel = self.body_angvel[
            :, self.fingertip_centered_body_id_env, 0:3
        ]
        self.fingertip_centered_jacobian = self.jacobian[
            :, self.fingertip_centered_body_id_env_actor - 1, 0:6, 0:7
        ]  # minus 1 because base is fixed

        self.fingertip_midpoint_pos = (
            self.fingertip_centered_pos.detach().clone()
        )  # initial value
        self.fingertip_midpoint_quat = self.fingertip_centered_quat  # always equal
        self.fingertip_midpoint_linvel = (
            self.fingertip_centered_linvel.detach().clone()
        )  # initial value
        # From sum of angular velocities (https://physics.stackexchange.com/questions/547698/understanding-addition-of-angular-velocity),
        # angular velocity of midpoint w.r.t. world is equal to sum of
        # angular velocity of midpoint w.r.t. hand and angular velocity of hand w.r.t. world.
        # Midpoint is in sliding contact (i.e., linear relative motion) with hand; angular velocity of midpoint w.r.t. hand is zero.
        # Thus, angular velocity of midpoint w.r.t. world is equal to angular velocity of hand w.r.t. world.
        self.fingertip_midpoint_angvel = self.fingertip_centered_angvel  # always equal
        self.fingertip_midpoint_jacobian = (
            self.left_finger_jacobian + self.right_finger_jacobian
        ) * 0.5  # approximation

        self.dof_torque = torch.zeros(
            (self.num_envs, self.num_dofs), device=self.device
        )
        self.fingertip_contact_wrench = torch.zeros(
            (self.num_envs, 6), device=self.device
        )

        self.ctrl_target_fingertip_centered_pos = torch.zeros(
            (self.num_envs, 3), device=self.device
        )
        self.ctrl_target_fingertip_centered_quat = torch.zeros(
            (self.num_envs, 4), device=self.device
        )
        self.ctrl_target_fingertip_midpoint_pos = torch.zeros(
            (self.num_envs, 3), device=self.device
        )
        self.ctrl_target_fingertip_midpoint_quat = torch.zeros(
            (self.num_envs, 4), device=self.device
        )
        self.ctrl_target_dof_pos = torch.zeros(
            (self.num_envs, self.num_dofs), device=self.device
        )
        self.ctrl_target_gripper_dof_pos = torch.zeros(
            (self.num_envs, 2), device=self.device
        )
        self.ctrl_target_fingertip_contact_wrench = torch.zeros(
            (self.num_envs, 6), device=self.device
        )

        self.prev_actions = torch.zeros(
            (self.num_envs, self.num_actions), device=self.device
        )

    def _set_dof_pos_target(self):
        """Set Kuka DOF position target to move fingertips towards target pose."""
        self.ctrl_target_dof_pos = fcfix.compute_dof_pos_target(
            cfg_ctrl=self.cfg_ctrl,
            arm_dof_pos=self.arm_dof_pos,
            fingertip_midpoint_pos=self.fingertip_centered_pos,
            fingertip_midpoint_quat=self.fingertip_centered_quat,
            jacobian=self.fingertip_midpoint_jacobian_tf,
            ctrl_target_fingertip_midpoint_pos=self.ctrl_target_fingertip_centered_pos,
            ctrl_target_fingertip_midpoint_quat=self.ctrl_target_fingertip_centered_quat,
            device=self.device)
        self.gym.set_dof_position_target_tensor_indexed(self.sim,
                                                        gymtorch.unwrap_tensor(self.ctrl_target_dof_pos),
                                                        gymtorch.unwrap_tensor(self.kuka_actor_ids_sim),
                                                        len(self.kuka_actor_ids_sim))
    def _set_dof_torque(self):
        """Set Kuka DOF torque to move fingertips towards target pose."""
        self.dof_torque = fcfix.compute_dof_torque(
            cfg_ctrl=self.cfg_ctrl,
            dof_vel=self.dof_vel,
            fingertip_midpoint_pos=self.fingertip_centered_pos,
            fingertip_midpoint_quat=self.fingertip_centered_quat,
            fingertip_midpoint_linvel=self.fingertip_centered_linvel,
            fingertip_midpoint_angvel=self.fingertip_centered_angvel,
            left_finger_force=self.left_finger_force,
            right_finger_force=self.right_finger_force,
            jacobian=self.fingertip_midpoint_jacobian_tf,
            arm_mass_matrix=self.arm_mass_matrix,
            ctrl_target_fingertip_midpoint_pos=self.ctrl_target_fingertip_centered_pos,
            ctrl_target_fingertip_midpoint_quat=self.ctrl_target_fingertip_centered_quat,
            ctrl_target_fingertip_contact_wrench=self.ctrl_target_fingertip_contact_wrench,
            device=self.device)
        self.gym.set_dof_actuation_force_tensor_indexed(self.sim,
                                                        gymtorch.unwrap_tensor(self.dof_torque),
                                                        gymtorch.unwrap_tensor(self.kuka_actor_ids_sim),
                                                        len(self.kuka_actor_ids_sim))
