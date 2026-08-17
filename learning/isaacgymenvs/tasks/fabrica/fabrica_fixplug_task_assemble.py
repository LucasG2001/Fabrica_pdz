import hydra
import numpy as np
import omegaconf
import os
import torch
import warp as wp
import json
import h5py
import pickle

from isaacgym import gymapi, gymtorch, torch_utils
from isaacgymenvs.tasks.fabrica.fabrica_fixplug_env import FabricaFixPlugEnv
from isaacgymenvs.tasks.fabrica.fabrica_task_assemble import FabricaTaskAssemble
from isaacgymenvs.tasks.fabrica.fabrica_algo_utils import closest_point_on_path
from isaacgymenvs.utils import torch_jit_utils


class FabricaFixPlugTaskAssemble(FabricaFixPlugEnv, FabricaTaskAssemble):

    def pre_physics_step(self, actions):
        """Reset environments. Apply actions from policy as position/rotation targets, force/torque targets, and/or PD gains."""

        env_ids = self.reset_buf.nonzero(as_tuple=False).squeeze(-1)
        if len(env_ids) > 0:
            self.reset_idx(env_ids)

        self._actions = actions.clone().to(self.device)  # shape = (num_envs, num_actions); values = [-1, 1]
        
        self._actions = self._preprocess_actions(self._actions)
        self._apply_actions_as_ctrl_targets(actions=self._actions,
                                            do_scale=True)

    def post_physics_step(self):
        """Step buffers. Refresh tensors. Compute observations and reward."""

        self.progress_buf[:] += 1

        self.refresh_base_tensors()
        self.refresh_env_tensors()
        self._refresh_task_tensors()

        self.compute_observations()
        self.compute_reward()

    def reset_idx(self, env_ids):
        """Reset specified environments."""

        self._reset_kuka(env_ids)
        self._reset_object(env_ids)

        self.reset_buf[env_ids] = 0
        self.progress_buf[env_ids] = 0

        self.insertion_successes[env_ids] = torch.zeros((len(env_ids),), dtype=torch.bool, device=self.device)
        self.deviations[env_ids] = torch.zeros((len(env_ids),), dtype=torch.bool, device=self.device)

        self.ctrl_target_fingertip_centered_quat_true[env_ids] = self.fingertip_centered_quat[env_ids].clone().detach()

    def _reset_kuka(self, env_ids):
        """Reset DOF states and DOF targets of Kuka."""

        # shape of dof_pos = (num_envs, num_dofs)
        # shape of dof_vel = (num_envs, num_dofs)

        # Initialize Kuka 
        self.dof_pos[env_ids] = self.arm_dof_pos_preassembly[env_ids, 0:7].clone().detach()
        self.dof_vel[env_ids] = 0.0  # shape = (num_envs, num_dofs)
        self.ctrl_target_dof_pos[env_ids] = self.arm_dof_pos_preassembly[env_ids, 0:7].clone().detach()

        multi_env_ids_int32 = self.kuka_actor_ids_sim[env_ids].flatten()
        self.gym.set_dof_state_tensor_indexed(self.sim,
                                              gymtorch.unwrap_tensor(self.dof_state),
                                              gymtorch.unwrap_tensor(multi_env_ids_int32),
                                              len(multi_env_ids_int32))

        # Set DOF torque
        self.gym.set_dof_actuation_force_tensor_indexed(self.sim,
                                                gymtorch.unwrap_tensor(torch.zeros_like(self.dof_torque)),
                                                gymtorch.unwrap_tensor(self.kuka_actor_ids_sim),
                                                len(self.kuka_actor_ids_sim))

        self.simulate_and_refresh()

    def _reset_object(self, env_ids):
        """Reset plug and socket."""

        self._reset_socket(env_ids)

    def _apply_actions_as_ctrl_targets(self, actions, do_scale):
        """Apply actions from policy as position/rotation targets."""

        # Interpret actions as target pos displacements and set pos target
        pos_actions = actions[:, 0:3]

        # NOTE: for debugging only
        # true_actions = self.socket_pos_real - self.plug_pos
        # true_actions_norm = torch.linalg.norm(true_actions, dim=-1, keepdim=True)
        # true_actions /= true_actions_norm
        # pos_actions = true_actions

        if do_scale:
            pos_actions = pos_actions @ torch.diag(torch.tensor(self.cfg_task.rl.pos_action_scale, device=self.device))

        # Adjust target plug pos to be close to the disassembly path
        # target_plug_pos = self.plug_pos + pos_actions
        # _, _, adjusted_target_plug_pos = closest_point_on_path(self.disassembly_path[:, :, 0:3], target_plug_pos, threshold=self.cfg_task.rl.close_error_thresh)
        # pos_actions = adjusted_target_plug_pos - self.plug_pos
        
        self.ctrl_target_fingertip_centered_pos = self.fingertip_centered_pos + pos_actions

        self.ctrl_target_fingertip_centered_quat = self.ctrl_target_fingertip_centered_quat_true.clone().detach()

        self.generate_ctrl_signals()
        
    def step(self, actions: torch.Tensor):
        return FabricaFixPlugEnv.step(self, actions)
