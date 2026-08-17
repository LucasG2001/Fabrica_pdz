"""Fabrica: class for peg insertion task.

Inherits Fabrica environment class and Factory abstract task class (not enforced).

Fabrica assets are loaded without any RL training.

Can be executed with python train.py task=FabricaTaskAsset.
"""


import hydra
import numpy as np
import omegaconf
import os
import torch
import warp as wp

from isaacgym import gymapi, gymtorch, torch_utils
from isaacgymenvs.tasks.factory.factory_schema_class_task import FactoryABCTask
from isaacgymenvs.tasks.factory.factory_schema_config_task import (
    FactorySchemaConfigTask,
)
from isaacgymenvs.tasks.fabrica.fabrica_env import FabricaEnv
from isaacgymenvs.utils import torch_jit_utils


class FabricaTaskAsset(FabricaEnv, FactoryABCTask):
    def __init__(
        self,
        cfg,
        rl_device,
        sim_device,
        graphics_device_id,
        headless,
        virtual_screen_capture,
        force_render,
    ):
        """Initialize instance variables. Initialize task superclass."""

        self.cfg = cfg
        self._get_task_yaml_params()

        super().__init__(
            cfg,
            rl_device,
            sim_device,
            graphics_device_id,
            headless,
            virtual_screen_capture,
            force_render,
        )

        self._acquire_task_tensors()
        self.parse_controller_spec()

        if self.viewer != None:
            self._set_viewer_params()

    def _get_task_yaml_params(self):
        """Initialize instance variables from YAML files."""

        cs = hydra.core.config_store.ConfigStore.instance()
        cs.store(name="factory_schema_config_task", node=FactorySchemaConfigTask)

        self.cfg_task = omegaconf.OmegaConf.create(self.cfg)
        self.max_episode_length = (
            self.cfg_task.rl.max_episode_length
        )  # required instance var for VecTask

        ppo_path = os.path.join(
            "train/FabricaTaskAssetPPO.yaml"
        )  # relative to Gym's Hydra search path (cfg dir)
        self.cfg_ppo = hydra.compose(config_name=ppo_path)
        self.cfg_ppo = self.cfg_ppo["train"]  # strip superfluous nesting

    def _acquire_task_tensors(self):
        """Acquire tensors."""
        pass

    def _refresh_task_tensors(self):
        """Refresh tensors."""
        pass

    def pre_physics_step(self, actions):
        """Reset environments. Apply actions from policy as position/rotation targets, force/torque targets, and/or PD gains."""

        env_ids = self.reset_buf.nonzero(as_tuple=False).squeeze(-1)
        if len(env_ids) > 0:
            self.reset_idx(env_ids)

        self._actions = actions.clone().to(self.device)  # shape = (num_envs, num_actions); values = [-1, 1]

    def post_physics_step(self):
        """Step buffers. Refresh tensors. Compute observations and reward."""

        self.progress_buf[:] += 1

        self.refresh_base_tensors()
        self.refresh_env_tensors()
        self._refresh_task_tensors()
        self.compute_observations()
        self.compute_reward()

    def compute_observations(self):
        """Compute observations."""

        return self.obs_buf  # shape = (num_envs, num_observations)

    def compute_reward(self):
        """Detect successes and failures. Update reward and reset buffers."""

        self._update_rew_buf()
        self._update_reset_buf()

    def _update_rew_buf(self):
        """Compute reward at current timestep."""
        pass

    def _update_reset_buf(self):
        """Assign environments for reset if successful or failed."""
        pass

    def reset_idx(self, env_ids):
        """Reset specified environments."""

        self._reset_kuka(env_ids)
        self._reset_object(env_ids)

        self.reset_buf[env_ids] = 0
        self.progress_buf[env_ids] = 0

    def _reset_kuka(self, env_ids):
        """Reset DOF states and DOF targets of Kuka."""

        # shape of dof_pos = (num_envs, num_dofs)
        # shape of dof_vel = (num_envs, num_dofs)

        # Initialize Kuka to rest pose
        self.dof_pos[:, 0:self.kuka_num_dofs] = torch.cat(
            (torch.tensor(self.cfg_base.env.kuka_rest_dof_pos, device=self.device),
             torch.tensor([self.asset_info_kuka_table.kuka_gripper_width_max], device=self.device),
             torch.tensor([self.asset_info_kuka_table.kuka_gripper_width_max], device=self.device)),
            dim=-1).unsqueeze(0).repeat((self.num_envs, 1))  # shape = (num_envs, num_dofs)

        self.dof_vel[env_ids, 0:self.kuka_num_dofs] = 0.0

        kuka_actor_ids_sim_int32 = self.kuka_actor_ids_sim.to(dtype=torch.int32, device=self.device)[env_ids]
        self.gym.set_dof_state_tensor_indexed(self.sim,
                                              gymtorch.unwrap_tensor(self.dof_state),
                                              gymtorch.unwrap_tensor(kuka_actor_ids_sim_int32),
                                              len(kuka_actor_ids_sim_int32))

        self.ctrl_target_dof_pos[env_ids, 0:self.kuka_num_dofs] = self.dof_pos[env_ids, 0:self.kuka_num_dofs]
        self.gym.set_dof_position_target_tensor(self.sim, gymtorch.unwrap_tensor(self.ctrl_target_dof_pos))

    def _reset_object(self, env_ids):
        """Reset root state of plug."""

        # shape of root_pos = (num_envs, num_actors, 3)
        # shape of root_quat = (num_envs, num_actors, 4)
        # shape of root_linvel = (num_envs, num_actors, 3)
        # shape of root_angvel = (num_envs, num_actors, 3)
        self.root_pos[env_ids, self.plug_actor_id_env] = torch.tensor(self.cfg_base.env.assembly_center, device=self.device) + \
            torch.tensor([0.0, 0.0, self.cfg_base.env.table_height], device=self.device)
        
        self.root_quat[env_ids, self.plug_actor_id_env] = torch.tensor([0.0, 0.0, 0.0, 1.0], device=self.device)

        self.root_linvel[env_ids, self.plug_actor_id_env] = 0.0
        self.root_angvel[env_ids, self.plug_actor_id_env] = 0.0

        plug_actor_ids_sim_int32 = self.plug_actor_ids_sim.to(dtype=torch.int32, device=self.device)
        self.gym.set_actor_root_state_tensor_indexed(self.sim,
                                                     gymtorch.unwrap_tensor(self.root_state),
                                                     gymtorch.unwrap_tensor(plug_actor_ids_sim_int32[env_ids]),
                                                     len(plug_actor_ids_sim_int32[env_ids]))

    def _reset_buffers(self, env_ids):
        """Reset buffers. """

        self.reset_buf[env_ids] = 0
        self.progress_buf[env_ids] = 0

    def _set_viewer_params(self):
        """Set viewer parameters."""

        cam_pos = gymapi.Vec3(-1.0, -1.0, 1.0)
        cam_target = gymapi.Vec3(0.0, 0.0, 0.5)
        self.gym.viewer_camera_look_at(self.viewer, None, cam_pos, cam_target)
