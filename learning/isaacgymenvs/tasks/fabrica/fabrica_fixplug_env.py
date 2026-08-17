import hydra
import math
import numpy as np
import os
import torch

from isaacgym import gymapi
from isaacgymenvs.tasks.fabrica.fabrica_fixplug_base import FabricaFixPlugBase
from isaacgymenvs.tasks.fabrica.fabrica_env import FabricaEnv


class FabricaFixPlugEnv(FabricaFixPlugBase, FabricaEnv):

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
        kuka_assets = self.import_kuka_assets()
        table_asset = self.import_table_assets()
        socket_assets = self._import_env_assets()
        self._create_actors(
            lower,
            upper,
            num_per_row,
            kuka_assets,
            socket_assets,
            table_asset,
        )

    def import_kuka_assets(self):
        """Set Kuka and table asset options. Import assets."""

        urdf_root = os.path.join(
            os.path.dirname(__file__), "..", "..", "..", "assets", "fabrica", "urdf", self.cfg_env.env.kuka_urdf_dir
        )

        kuka_options = gymapi.AssetOptions()
        # See fabrica_base.py's import_kuka_assets for why this must be False (not the
        # Franka-copied True): fabrica_kuka.urdf's <visual> and <collision> geoms are the same
        # .STL file per link, so flipping only the visual tears it away from the correct
        # (unflipped) collision body -- verified headless, True produces disjoint floating links.
        kuka_options.flip_visual_attachments = False
        kuka_options.fix_base_link = True
        kuka_options.collapse_fixed_joints = False
        kuka_options.thickness = 0.0  # default = 0.02
        kuka_options.density = 1000.0  # default = 1000.0
        kuka_options.armature = 0.01  # default = 0.0
        kuka_options.use_physx_armature = True
        if self.cfg_base.sim.add_damping:
            kuka_options.linear_damping = (
                1.0  # default = 0.0; increased to improve stability
            )
            kuka_options.max_linear_velocity = (
                1.0  # default = 1000.0; reduced to prevent CUDA errors
            )
            kuka_options.angular_damping = (
                5.0  # default = 0.5; increased to improve stability
            )
            kuka_options.max_angular_velocity = (
                2 * math.pi
            )  # default = 64.0; reduced to prevent CUDA errors
        else:
            kuka_options.linear_damping = 0.0  # default = 0.0
            kuka_options.max_linear_velocity = 1.0  # default = 1000.0
            kuka_options.angular_damping = 0.5  # default = 0.5
            kuka_options.max_angular_velocity = 2 * math.pi  # default = 64.0
        kuka_options.disable_gravity = True
        kuka_options.enable_gyroscopic_forces = True
        kuka_options.default_dof_drive_mode = gymapi.DOF_MODE_NONE
        kuka_options.use_mesh_materials = True
        if self.cfg_base.mode.export_scene:
            kuka_options.mesh_normal_mode = gymapi.COMPUTE_PER_FACE

        assemblies = self.cfg_env.env.assemblies
        part_plug, part_socket = self.cfg_env.env.part_plug, self.cfg_env.env.part_socket
        assert (part_plug is None and part_socket is None) or (part_plug is not None and part_socket is not None)

        self.part_names = {'assembly': [], 'plug': [], 'socket': []}
        kuka_assets = {}
        for assembly in assemblies:
            kuka_assets[assembly] = {}
            
            if part_plug is not None and part_socket is not None:
                part_pairs = [{'plug': str(part_plug), 'socket': str(part_socket)}]
            else:
                part_pairs = list(self.assembly_pairs[assembly].values())

            for part_pair in part_pairs:
                self.part_names['assembly'].append(assembly)
                self.part_names['plug'].append(part_pair['plug'])
                self.part_names['socket'].append(part_pair['socket'])

                kuka_file = f"{assembly}_{part_pair['plug']}_{part_pair['socket']}.urdf"
                kuka_asset = self.gym.load_asset(
                    self.sim, urdf_root, kuka_file, kuka_options
                )
                kuka_assets[assembly][(part_pair['plug'], part_pair['socket'])] = kuka_asset
        
        return kuka_assets

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

        socket_assets = []
        for assembly in assemblies:
            if part_plug is not None and part_socket is not None:
                part_pairs = [{'plug': str(part_plug), 'socket': str(part_socket)}]
            else:
                part_pairs = list(self.assembly_pairs[assembly].values())
            for part_pair in part_pairs:
                component = part_pair['socket']
                part_file = (
                    self.asset_info_fabrica[assembly][component]["urdf_path"]
                    + ".urdf"
                )
                part_options.density = self.asset_info_fabrica[assembly][
                    component
                ]["density"]
                part_options.fix_base_link = True
                part_asset = self.gym.load_asset(
                    self.sim, urdf_root, part_file, part_options
                )
                socket_assets.append(part_asset)

        return socket_assets

    def _create_actors(
        self,
        lower,
        upper,
        num_per_row,
        kuka_assets,
        socket_assets,
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
        self.socket_handles = []
        self.shape_ids = []
        self.asset_indices = []
        self.kuka_actor_ids_sim = []  # within-sim indices
        self.table_actor_ids_sim = []  # within-sim indices
        self.socket_actor_ids_sim = []  # within-sim indices
        actor_count = 0

        for i in range(self.num_envs):

            env_ptr = self.gym.create_env(self.sim, lower, upper, num_per_row)

            j = i % len(self.part_names['assembly'])
            assembly = self.part_names['assembly'][j]
            part_plug, part_socket = self.part_names['plug'][j], self.part_names['socket'][j]

            kuka_asset = kuka_assets[assembly][(part_plug, part_socket)]
            if self.cfg_env.sim.disable_kuka_collisions:
                kuka_handle = self.gym.create_actor(env_ptr, kuka_asset, kuka_pose, 'kuka', i + self.num_envs,
                                                      0, 0)
            else:
                kuka_handle = self.gym.create_actor(env_ptr, kuka_asset, kuka_pose, 'kuka', i, 0, 0)
            self.kuka_actor_ids_sim.append(actor_count)
            actor_count += 1

            part_pose = gymapi.Transform()
            part_pose.p.x = self.cfg_base.env.assembly_center[0]
            part_pose.p.y = self.cfg_base.env.assembly_center[1]
            part_pose.p.z = self.cfg_base.env.assembly_center[2] + self.cfg_base.env.table_height
            part_pose.r = gymapi.Quat(0.0, 0.0, 0.0, 1.0)
            part_handle = self.gym.create_actor(env_ptr, socket_assets[j], part_pose, 'socket', i, 0, 0)
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
            plug_id = self.gym.find_actor_rigid_body_index(env_ptr, kuka_handle, part_plug, gymapi.DOMAIN_ACTOR)
            self.shape_ids = [link7_id, hand_id, left_finger_id, right_finger_id, plug_id]

            kuka_shape_props = self.gym.get_actor_rigid_shape_properties(env_ptr, kuka_handle)
            for shape_id in self.shape_ids:
                kuka_shape_props[shape_id].friction = self.cfg_env.env.kuka_friction
                kuka_shape_props[shape_id].rolling_friction = 0.0  # default = 0.0
                kuka_shape_props[shape_id].torsion_friction = 0.0  # default = 0.0
                kuka_shape_props[shape_id].restitution = 0.0  # default = 0.0
                kuka_shape_props[shape_id].compliance = 0.0  # default = 0.0
                kuka_shape_props[shape_id].thickness = 0.0  # default = 0.0
            self.gym.set_actor_rigid_shape_properties(env_ptr, kuka_handle, kuka_shape_props)

            part_shape_props = self.gym.get_actor_rigid_shape_properties(env_ptr, part_handle)
            part_shape_props[0].friction = self.asset_info_fabrica[assembly][part_socket]['friction']
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
            self.socket_handles.append(part_handle)
            self.asset_indices.append(j)

        self.num_actors = int(actor_count / self.num_envs)  # per env
        self.num_bodies = self.gym.get_env_rigid_body_count(env_ptr)  # per env
        self.num_dofs = self.gym.get_env_dof_count(env_ptr)  # per env

        # For setting targets
        self.kuka_actor_ids_sim = torch.tensor(self.kuka_actor_ids_sim, dtype=torch.int32, device=self.device)
        self.socket_actor_ids_sim = torch.tensor(self.socket_actor_ids_sim, dtype=torch.int32, device=self.device)

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
        self.plug_body_id_env = self.gym.find_actor_rigid_body_index(env_ptr, kuka_handle, part_plug, gymapi.DOMAIN_ENV)

        self.hand_body_id_env_actor = self.gym.find_actor_rigid_body_index(env_ptr, kuka_handle, 'kuka_hand',
                                                                     gymapi.DOMAIN_ACTOR)

        self.left_finger_body_id_env_actor = self.gym.find_actor_rigid_body_index(env_ptr, kuka_handle, 'kuka_leftfinger',
                                                                            gymapi.DOMAIN_ACTOR)
        self.right_finger_body_id_env_actor = self.gym.find_actor_rigid_body_index(env_ptr, kuka_handle,
                                                                             'kuka_rightfinger', gymapi.DOMAIN_ACTOR)
        self.fingertip_centered_body_id_env_actor = self.gym.find_actor_rigid_body_index(env_ptr, kuka_handle,
                                                                                    'kuka_fingertip_centered',
                                                                                   gymapi.DOMAIN_ACTOR)
        self.plug_body_id_env_actor = self.gym.find_actor_rigid_body_index(env_ptr, kuka_handle, part_plug, gymapi.DOMAIN_ACTOR)

    def _acquire_env_tensors(self):
        """Acquire and wrap tensors. Create views."""

        self.plug_pos = self.body_pos[:, self.plug_body_id_env, 0:3]
        self.plug_quat = self.body_quat[:, self.plug_body_id_env, 0:4]
        self.plug_linvel = self.body_linvel[:, self.plug_body_id_env, 0:3]
        self.plug_angvel = self.body_angvel[:, self.plug_body_id_env, 0:3]

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
    