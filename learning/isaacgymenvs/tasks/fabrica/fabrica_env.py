"""Fabrica: class for pegs environment.

Inherits Fabrica base class and Factory abstract environment class. Inherited by Fabrica asset/assemble/disassemble task class. Not directly executed.

Configuration defined in FabricaEnv.yaml. Asset info defined in fabrica_asset_info.yaml.
"""


import hydra
import math
import numpy as np
import os
import torch

from isaacgym import gymapi
from isaacgymenvs.tasks.factory.factory_schema_class_env import FactoryABCEnv
from isaacgymenvs.tasks.factory.factory_schema_config_env import FactorySchemaConfigEnv
from isaacgymenvs.tasks.fabrica.fabrica_base import FabricaBase


class FabricaEnv(FabricaBase, FactoryABCEnv):
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
        """Initialize instance variables. Initialize environment superclass. Acquire tensors."""

        self._get_env_yaml_params()

        super().__init__(
            cfg,
            rl_device,
            sim_device,
            graphics_device_id,
            headless,
            virtual_screen_capture,
            force_render,
        )

        self.acquire_base_tensors()  # defined in superclass
        self._acquire_env_tensors()
        self.refresh_base_tensors()  # defined in superclass
        self.refresh_env_tensors()

    def _get_env_yaml_params(self):
        """Initialize instance variables from YAML files."""

        cs = hydra.core.config_store.ConfigStore.instance()
        cs.store(name="factory_schema_config_env", node=FactorySchemaConfigEnv)

        config_path = "task/FabricaEnv.yaml"  # relative to Gym's Hydra search path (cfg dir)
        self.cfg_env = hydra.compose(config_name=config_path)
        self.cfg_env = self.cfg_env["task"]  # strip superfluous nesting

        # TODO: check
        # self.cfg_env.env.assemblies = self.cfg_task.env.assemblies

        asset_info_path = "../../assets/fabrica/yaml/fabrica_asset_info/fabrica.yaml"  # relative to Gym's Hydra search path (cfg dir)
        self.asset_info_fabrica = hydra.compose(config_name=asset_info_path)
        self.asset_info_fabrica = self.asset_info_fabrica[""][""][""][""][""][""][
            "assets"
        ]["fabrica"][
            "yaml"
        ]["fabrica_asset_info"]  # strip superfluous nesting

        assembly_pair_path = f"../../assets/fabrica/yaml/fabrica_asset_info/{self.cfg_env.env.yaml_pair_name}.yaml"
        self.assembly_pairs = hydra.compose(config_name=assembly_pair_path)
        self.assembly_pairs = self.assembly_pairs[""][""][""][""][""][""][
            "assets"
        ]["fabrica"][
            "yaml"
        ]["fabrica_asset_info"]

        # NOTE: task cfg overwrite env cfg
        if hasattr(self.cfg_task.env, "assemblies") and self.cfg_task.env.assemblies is not None:
            self.cfg_env.env.assemblies = self.cfg_task.env.assemblies
        if hasattr(self.cfg_task.env, "assemblies_exclude") and self.cfg_task.env.assemblies_exclude is not None:
            for assembly_exclude in self.cfg_task.env.assemblies_exclude:
                self.cfg_env.env.assemblies.remove(assembly_exclude)
        if hasattr(self.cfg_task.env, "part_plug") and self.cfg_task.env.part_plug is not None:
            self.cfg_env.env.part_plug = self.cfg_task.env.part_plug
        if hasattr(self.cfg_task.env, "part_socket") and self.cfg_task.env.part_socket is not None:
            self.cfg_env.env.part_socket = self.cfg_task.env.part_socket
        if hasattr(self.cfg_task.env, "kuka_friction") and self.cfg_task.env.kuka_friction is not None:
            self.cfg_env.env.kuka_friction = self.cfg_task.env.kuka_friction

    def create_envs(self):
        """Set env options. Import assets. Create actors."""

        lower = gymapi.Vec3(
            -self.cfg_base.env.env_spacing, -self.cfg_base.env.env_spacing, 0.0
        )
        upper = gymapi.Vec3(
            self.cfg_base.env.env_spacing,
            self.cfg_base.env.env_spacing,
            self.cfg_base.env.env_spacing,
        )
        num_per_row = int(np.sqrt(self.num_envs))

        self.print_sdf_warning()
        kuka_asset, table_asset = self.import_kuka_assets()
        part_assets = self._import_env_assets()
        self._create_actors(
            lower,
            upper,
            num_per_row,
            kuka_asset,
            part_assets,
            table_asset,
        )

    def _import_env_assets(self):
        """Set part asset options. Import assets."""

        urdf_root = os.path.join(
            os.path.dirname(__file__), "..", "..", "..", "assets", "fabrica", "urdf", "fabrica"
        )

        part_options = gymapi.AssetOptions()
        part_options.flip_visual_attachments = False
        part_options.fix_base_link = False
        part_options.thickness = 0.0  # default = 0.02
        part_options.armature = 0.0  # default = 0.0
        part_options.use_physx_armature = True
        part_options.linear_damping = 0.5  # default = 0.0
        part_options.max_linear_velocity = 1000.0  # default = 1000.0
        part_options.angular_damping = 0.5  # default = 0.5
        part_options.max_angular_velocity = 64.0  # default = 64.0
        part_options.disable_gravity = False
        part_options.enable_gyroscopic_forces = True
        part_options.default_dof_drive_mode = gymapi.DOF_MODE_NONE
        part_options.use_mesh_materials = False
        if self.cfg_base.mode.export_scene:
            part_options.mesh_normal_mode = gymapi.COMPUTE_PER_FACE

        assemblies = self.cfg_env.env.assemblies
        part_plug, part_socket = self.cfg_env.env.part_plug, self.cfg_env.env.part_socket
        assert (part_plug is None and part_socket is None) or (part_plug is not None and part_socket is not None)

        self.part_names = {'assembly': [], 'plug': [], 'socket': []}
        part_assets = {'plug': [], 'socket': []}

        for assembly in assemblies:
            if part_plug is not None and part_socket is not None:
                part_pairs = [{'plug': str(part_plug), 'socket': str(part_socket)}]
            else:
                part_pairs = list(self.assembly_pairs[assembly].values())
            for part_pair in part_pairs:
                self.part_names['assembly'].append(assembly)
                for part_type, component in part_pair.items():
                    part_file = (
                        self.asset_info_fabrica[assembly][component]["urdf_path"]
                        + ".urdf"
                    )
                    part_options.density = self.asset_info_fabrica[assembly][
                        component
                    ]["density"]
                    if part_type == 'plug':
                        part_options.fix_base_link = False
                    elif part_type == 'socket':
                        part_options.fix_base_link = True
                    part_asset = self.gym.load_asset(
                        self.sim, urdf_root, part_file, part_options
                    )
                    part_assets[part_type].append(part_asset)
                    self.part_names[part_type].append(component)

        return part_assets

    def _create_actors(
        self,
        lower,
        upper,
        num_per_row,
        kuka_asset,
        part_assets,
        table_asset,
    ):
        """Set initial actor poses. Create actors. Set shape and DOF properties."""
        # NOTE: Closely adapted from FactoryEnvInsertion; however, plug grasp offsets, plug widths, socket heights,
        # and asset indices are now stored for possible use during policy learning.""" # TODO: update

        kuka_pose = gymapi.Transform()
        kuka_pose.p.x = self.cfg_base.env.kuka_pos[0]
        kuka_pose.p.y = self.cfg_base.env.kuka_pos[1]
        kuka_pose.p.z = self.cfg_base.env.table_height + self.cfg_base.env.kuka_pos[2]
        kuka_pose.r = gymapi.Quat.from_euler_zyx(*self.cfg_base.env.kuka_euler)

        table_pose = gymapi.Transform()
        table_pose.p.x = 0.0
        table_pose.p.y = 0.0
        table_pose.p.z = self.cfg_base.env.table_height * 0.5
        table_pose.r = gymapi.Quat(0.0, 0.0, 0.0, 1.0)

        self.env_ptrs = []
        self.kuka_handles = []
        self.table_handles = []
        self.part_handles = []
        self.shape_ids = []
        self.asset_indices = []
        self.kuka_actor_ids_sim = []  # within-sim indices
        self.table_actor_ids_sim = []  # within-sim indices
        self.plug_actor_ids_sim = []  # within-sim indices
        self.socket_actor_ids_sim = []  # within-sim indices
        actor_count = 0

        for i in range(self.num_envs):

            env_ptr = self.gym.create_env(self.sim, lower, upper, num_per_row)

            if self.cfg_env.sim.disable_kuka_collisions:
                kuka_handle = self.gym.create_actor(env_ptr, kuka_asset, kuka_pose, 'kuka', i + self.num_envs,
                                                      0, 0)
            else:
                kuka_handle = self.gym.create_actor(env_ptr, kuka_asset, kuka_pose, 'kuka', i, 0, 0)
            self.kuka_actor_ids_sim.append(actor_count)
            actor_count += 1

            j = i % len(self.part_names['assembly'])
            assembly = self.part_names['assembly'][j]
            part_plug, part_socket = self.part_names['plug'][j], self.part_names['socket'][j]

            part_handles = []
            for part_type in ['plug', 'socket']:
                part_pose = gymapi.Transform()
                part_pose.p.x = self.cfg_base.env.assembly_center[0]
                part_pose.p.y = self.cfg_base.env.assembly_center[1]
                part_pose.p.z = self.cfg_base.env.assembly_center[2] + self.cfg_base.env.table_height
                part_pose.r = gymapi.Quat(0.0, 0.0, 0.0, 1.0)
                part_handle = self.gym.create_actor(env_ptr, part_assets[part_type][j], part_pose, part_type, i, 0, 0)
                part_handles.append(part_handle)
                if part_type == 'plug':
                    self.plug_actor_ids_sim.append(actor_count)
                elif part_type == 'socket':
                    self.socket_actor_ids_sim.append(actor_count)
                actor_count += 1

            table_handle = self.gym.create_actor(env_ptr, table_asset, table_pose, 'table', i, 0, 0)
            self.table_actor_ids_sim.append(actor_count)
            actor_count += 1

            link7_id = self.gym.find_actor_rigid_body_index(env_ptr, kuka_handle, 'kuka_link7', gymapi.DOMAIN_ACTOR)
            hand_id = self.gym.find_actor_rigid_body_index(env_ptr, kuka_handle, 'kuka_hand', gymapi.DOMAIN_ACTOR)
            left_finger_id = self.gym.find_actor_rigid_body_index(env_ptr, kuka_handle, 'kuka_leftfinger',
                                                                  gymapi.DOMAIN_ACTOR)
            right_finger_id = self.gym.find_actor_rigid_body_index(env_ptr, kuka_handle, 'kuka_rightfinger',
                                                                   gymapi.DOMAIN_ACTOR)
            self.shape_ids = [link7_id, hand_id, left_finger_id, right_finger_id]

            kuka_shape_props = self.gym.get_actor_rigid_shape_properties(env_ptr, kuka_handle)
            for shape_id in self.shape_ids:
                kuka_shape_props[shape_id].friction = self.cfg_base.env.kuka_friction
                kuka_shape_props[shape_id].rolling_friction = 0.0  # default = 0.0
                kuka_shape_props[shape_id].torsion_friction = 0.0  # default = 0.0
                kuka_shape_props[shape_id].restitution = 0.0  # default = 0.0
                kuka_shape_props[shape_id].compliance = 0.0  # default = 0.0
                kuka_shape_props[shape_id].thickness = 0.0  # default = 0.0
            self.gym.set_actor_rigid_shape_properties(env_ptr, kuka_handle, kuka_shape_props)

            for part_handle, component in zip(part_handles, [part_plug, part_socket]):
                part_shape_props = self.gym.get_actor_rigid_shape_properties(env_ptr, part_handle)
                part_shape_props[0].friction = self.asset_info_fabrica[assembly][component]['friction']
                part_shape_props[0].rolling_friction = 0.0  # default = 0.0
                part_shape_props[0].torsion_friction = 0.0  # default = 0.0
                part_shape_props[0].restitution = 0.0  # default = 0.0
                part_shape_props[0].compliance = 0.0  # default = 0.0
                part_shape_props[0].thickness = 0.0  # default = 0.0
                self.gym.set_actor_rigid_shape_properties(env_ptr, part_handle, part_shape_props)

            table_shape_props = self.gym.get_actor_rigid_shape_properties(env_ptr, table_handle)
            table_shape_props[0].friction = self.cfg_base.env.table_friction
            table_shape_props[0].rolling_friction = 0.0  # default = 0.0
            table_shape_props[0].torsion_friction = 0.0  # default = 0.0
            table_shape_props[0].restitution = 0.0  # default = 0.0
            table_shape_props[0].compliance = 0.0  # default = 0.0
            table_shape_props[0].thickness = 0.0  # default = 0.0
            self.gym.set_actor_rigid_shape_properties(env_ptr, table_handle, table_shape_props)

            self.kuka_num_dofs = self.gym.get_actor_dof_count(env_ptr, kuka_handle)

            self.gym.enable_actor_dof_force_sensors(env_ptr, kuka_handle)

            self.env_ptrs.append(env_ptr)
            self.kuka_handles.append(kuka_handle)
            self.table_handles.append(table_handle)
            self.part_handles.append(part_handles)
            self.asset_indices.append(j)

        self.num_actors = int(actor_count / self.num_envs)  # per env
        self.num_bodies = self.gym.get_env_rigid_body_count(env_ptr)  # per env
        self.num_dofs = self.gym.get_env_dof_count(env_ptr)  # per env

        # For setting targets
        self.kuka_actor_ids_sim = torch.tensor(self.kuka_actor_ids_sim, dtype=torch.int32, device=self.device)
        self.plug_actor_ids_sim = torch.tensor(self.plug_actor_ids_sim, dtype=torch.int32, device=self.device)
        self.socket_actor_ids_sim = torch.tensor(self.socket_actor_ids_sim, dtype=torch.int32, device=self.device)

        # For extracting root pos/quat
        self.part_actor_ids_env = []

        self.plug_actor_id_env = self.gym.find_actor_index(env_ptr, 'plug', gymapi.DOMAIN_ENV)
        self.socket_actor_id_env = self.gym.find_actor_index(env_ptr, 'socket', gymapi.DOMAIN_ENV)

        # For extracting body pos/quat, force, and Jacobian
        self.robot_base_body_id_env = self.gym.find_actor_rigid_body_index(env_ptr, kuka_handle,
                                                                                   'kuka_link0',
                                                                                   gymapi.DOMAIN_ENV)
        self.hand_body_id_env = self.gym.find_actor_rigid_body_index(env_ptr, kuka_handle, 'kuka_hand',
                                                                     gymapi.DOMAIN_ENV)
        self.left_finger_body_id_env = self.gym.find_actor_rigid_body_index(env_ptr, kuka_handle, 'kuka_leftfinger',
                                                                            gymapi.DOMAIN_ENV)
        self.right_finger_body_id_env = self.gym.find_actor_rigid_body_index(env_ptr, kuka_handle,
                                                                             'kuka_rightfinger', gymapi.DOMAIN_ENV)
        self.fingertip_centered_body_id_env = self.gym.find_actor_rigid_body_index(env_ptr, kuka_handle,
                                                                                   'kuka_fingertip_centered',
                                                                                   gymapi.DOMAIN_ENV)
        # NOTE: ignored part related

        self.hand_body_id_env_actor = self.gym.find_actor_rigid_body_index(env_ptr, kuka_handle, 'kuka_hand',
                                                                     gymapi.DOMAIN_ACTOR)

        self.left_finger_body_id_env_actor = self.gym.find_actor_rigid_body_index(env_ptr, kuka_handle, 'kuka_leftfinger',
                                                                            gymapi.DOMAIN_ACTOR)
        self.right_finger_body_id_env_actor = self.gym.find_actor_rigid_body_index(env_ptr, kuka_handle,
                                                                             'kuka_rightfinger', gymapi.DOMAIN_ACTOR)
        self.fingertip_centered_body_id_env_actor = self.gym.find_actor_rigid_body_index(env_ptr, kuka_handle,
                                                                                    'kuka_fingertip_centered',
                                                                                   gymapi.DOMAIN_ACTOR)


    def _acquire_env_tensors(self):
        """Acquire and wrap tensors. Create views."""

        self.plug_pos = self.root_pos[:, self.plug_actor_id_env, 0:3]
        self.plug_quat = self.root_quat[:, self.plug_actor_id_env, 0:4]
        self.plug_linvel = self.root_linvel[:, self.plug_actor_id_env, 0:3]
        self.plug_angvel = self.root_angvel[:, self.plug_actor_id_env, 0:3]

        self.socket_pos = self.root_pos[:, self.socket_actor_id_env, 0:3]
        self.socket_quat = self.root_quat[:, self.socket_actor_id_env, 0:4]
        self.socket_linvel = self.root_linvel[:, self.socket_actor_id_env, 0:3]
        self.socket_angvel = self.root_angvel[:, self.socket_actor_id_env, 0:3]

        # TODO: Define socket height and plug height params in asset info YAML.
        # self.plug_com_pos = self.translate_along_local_z(pos=self.plug_pos,
        #                                                  quat=self.plug_quat,
        #                                                  offset=self.socket_heights + self.plug_heights * 0.5,
        #                                                  device=self.device)
        self.plug_com_quat = self.plug_quat  # always equal
        # self.plug_com_linvel = self.plug_linvel + torch.cross(self.plug_angvel,
        #                                                       (self.plug_com_pos - self.plug_pos),
        #                                                       dim=1)
        self.plug_com_angvel = self.plug_angvel  # always equal

    def refresh_env_tensors(self):
        """Refresh tensors."""
        # NOTE: Tensor refresh functions should be called once per step, before setters.

        pass
