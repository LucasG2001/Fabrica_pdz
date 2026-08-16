"""Real-robot interface for Fabrica, targeting the dual-KUKA-LBR rig via
kukapy (franka_ros2_ws/src/kukapy) instead of the original Franka
Emika Panda + frankapy setup.

Porting notes (see kukapy's README.md for the full frankapy -> kukapy
method-mapping table, frame conventions, and known gaps):
  - `FrankaArm`/`FrankaConstants` -> `KukaArm`/`KukaConstants`. `robot_num`
    keeps its original meaning (1 = "left"/holder arm, 2 = "right"/inserter
    arm), now mapped onto lbr_one/lbr_two.
  - `init_joint_pose_publisher()`/`init_rl_publisher()`: frankapy's manual
    `rospy.Publisher(SensorDataGroup)` setup is gone -- `goto_joints(dynamic=True)`
    /`goto_pose(dynamic=True)` open the equivalent streaming session
    internally.
  - `publish_traj()`/`send_control_targets()`: the hand-rolled protobuf
    (`JointPositionSensorMessage`/`PosePositionSensorMessage`/
    `CartesianImpedanceSensorMessage`) + `make_sensor_group_msg()` +
    `rospy` publish is replaced by `self.fa.publish_streaming_joints(...)`
    / `self.fa.publish_streaming_pose(...)`.
  - `rospy.Rate`/`rospy.get_time()` -> plain `time.sleep()`/`time.time()`:
    `KukaArm` owns its own rclpy node/context internally, so this file no
    longer needs to talk to ROS directly.
  - `get_tool_jacobian()`/`calculate_tool_velocity()` are NOT currently
    usable: `KukaArm.get_jacobian()` raises `NotImplementedError` (no
    verified Jacobian source in this workspace -- see kukapy's README.md).
    Neither has a caller in this file today.
  - Requires the dual-arm rig to be brought up with
    `lbr_dual_arm_bringup/launch/cartesian_impedance.launch.py`, and the
    `franka_ros2_ws` ROS2 workspace to be sourced before running any
    `real_robot/*.py` script (see main README.md's "Install kukapy for
    real-robot experiments" section).
"""
import os
import sys

project_base_dir = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
sys.path.append(project_base_dir)

try:
    from kukapy import KukaArm
    from kukapy import KukaConstants as KC
except ImportError as exc:
    raise ImportError(
        "Could not import kukapy. Source the franka_ros2_ws ROS2 workspace before "
        "running real-robot scripts, e.g.:\n"
        "  source /opt/ros/<distro>/setup.bash\n"
        "  source ~/franka_ros2_ws/install/setup.bash\n"
        "See franka_ros2_ws/src/kukapy/README.md for details."
    ) from exc

from autolab_core import RigidTransform
from isaacgym import torch_utils
import isaacgymenvs
from isaacgymenvs.learning import common_player
from real_robot.algo_utils import closest_point_on_path, do_deltapos_path_transform, undo_deltapos_path_transform

import time
import numpy as np
from scipy.spatial.transform import Rotation as R
import yaml
from gym.spaces import Box
import torch
import pickle

import select
import tty
import termios


def get_key_nonblocking():
    if select.select([sys.stdin], [], [], 0)[0]:
        return sys.stdin.read(1)
    return None


def load_config():
    # Load config in real execution
    config_path = os.path.join(os.path.dirname(__file__), 'config.yaml')
    with open(config_path, 'r') as f:
        config = yaml.safe_load(f)
    return config


def load_rl_policy(checkpoint_path, device='cuda'):

    # Load config used in training
    config_path = os.path.join(os.path.dirname(checkpoint_path), '..', 'config.yaml')
    with open(config_path, "r") as f:
        sim_config = yaml.safe_load(f)

    env_info = {
        "observation_space": Box(
            low=-np.Inf,
            high=np.Inf,
            shape=(sim_config["task"]["env"]["numObservations"],),
            dtype=np.float32,
        ),
        "action_space": Box(
            low=-1.0,
            high=1.0,
            shape=(sim_config["task"]["env"]["numActions"],),
            dtype=np.float32,
        ),
    }
    sim_config["train"]["params"]["config"]["env_info"] = env_info
    sim_config["train"]["params"]["config"]["device_name"] = device

    # Restore policy from checkpoint
    policy = common_player.CommonPlayer(sim_config["train"]["params"])
    policy.restore(checkpoint_path)
    policy.reset()

    return policy


def load_disassembly_paths(sim_config):
    assemblies = sim_config['task']['env']['assemblies']
    disassembly_paths = {}
    isaacgymenvs_dir = os.path.dirname(isaacgymenvs.__file__)
    for assembly in assemblies:
        disassembly_paths[assembly] = {}
        plan_info_path = os.path.join(isaacgymenvs_dir, sim_config['task']['env']['data_dir'], 'plan_info', f'{assembly}.pkl')
        with open(plan_info_path, 'rb') as f:
            plan_info = pickle.load(f)
        for (part_plug, part_socket) in plan_info:
            disassembly_paths[assembly][part_plug] = plan_info[(part_plug, part_socket)]['path']
    return disassembly_paths


class RobotInterface:

    def __init__(self, robot_num, residual=False):
        self.robot_num = robot_num
        self.fa = KukaArm(with_gripper=True, robot_num=robot_num)
        self.device = 'cuda'

        self.joint_limits_min = KC.JOINT_LIMITS_MIN
        self.joint_limits_max = KC.JOINT_LIMITS_MAX

        self.tool_to_base = RigidTransform(
            # Translation updated to the KUKA Y-gripper's calibrated gripper_tcp offset
            # (0.1455m, from kuka_iiwa7_y_gripper.urdf's gripper_tcp_joint -- same source used
            # for fabrica_kuka.urdf's kuka_fingertip_centered_joint and the sim's
            # palm_to_finger_dist). Rotation is UNCHANGED from the original Franka port (a
            # 45-degree yaw matching panda_hand's frame convention) -- NOT verified against the
            # KUKA Y-gripper's actual mount rotation. Frame name strings are intentionally left
            # as "franka_tool"/"franka_tool_base": kukapy's get_pose()/goto_pose() use those
            # exact strings for drop-in compatibility (see kukapy README's "Frame conventions").
            translation=[0,0,0.1455],
            rotation=[[0.7071, 0.7071, 0], [-0.7071, 0.7071, 0], [0, 0, 1]],
            from_frame="franka_tool",
            to_frame="franka_tool_base")

        self.residual = residual

    def reset_arm(self, home_gripper=True):
        self.fa.goto_gripper(0.08, grasp=False)
        self.fa.reset_joints()
        if home_gripper:
            self.fa.home_gripper()

    def clip_joints(self, joints):
        d = np.clip(joints, self.joint_limits_min + 1e-3, self.joint_limits_max - 1e-3)
        norm = np.linalg.norm(d - joints)
        if norm > 1e-3:
            print(f"[WARNING]: Clipped joints, norm: {norm}")
        return d

    def init_joint_pose_publisher(self, T, init_joints):
        self.fa.goto_joints(self.clip_joints(init_joints),
                 duration=T,
                 dynamic=True,
                 buffer_time=10,
                 use_impedance=True,
                 ignore_virtual_walls=True,
                 ignore_errors=True,
        )

    def init_rl_publisher(self, real_config):
        T = real_config['control']['duration']
        self.fa.goto_pose(
            tool_pose=self.fa.get_pose(),
            duration=T,
            use_impedance=True,
            dynamic=True,
            buffer_time=T * 10,
            cartesian_impedances=real_config['control']['prop_gains'],
            ignore_virtual_walls=True,
            ignore_errors=True,
        )

    def publish_traj(self, joint_pose, dt):
        self.fa.publish_streaming_joints(self.clip_joints(joint_pose))
        time.sleep(dt)

    def goto_gripper(self, width, block=True, grasp=False, grasp_force=60):
        width_clipped = KC.GRIPPER_WIDTH_MIN + width * (KC.GRIPPER_WIDTH_MAX - KC.GRIPPER_WIDTH_MIN)
        self.fa.goto_gripper(
                     width_clipped,
                     grasp=grasp,
                     speed=0.01,
                     force=grasp_force if grasp else 10,
                     epsilon_inner=0.08,
                     epsilon_outer=0.08,
                     block=block,
                     ignore_errors=True,
                     skill_desc='GoToGripper')

    def goto_joints(self, joints, duration=5, ignore_virtual_walls=False):
        joints_clipped = self.clip_joints(joints)
        self.fa.goto_joints(
            joints_clipped,
            duration=duration,
            ignore_virtual_walls=ignore_virtual_walls)

    def stop_skill(self):
        if not self.fa.is_skill_done():
            self.fa.stop_skill()

    def guide_mode(self, duration, block=False, print_pose=False, translation_only=False):
        start_t = time.time()
        if translation_only:
            self.fa.selective_guidance_mode(duration, block=block, use_impedance=True, use_ee_frame=True,cartesian_impedances=[0.0, 0.0, 0.0, 15.0, 15.0, 15.0])
        else:
            self.fa.run_guide_mode(duration=duration, block=block)

        if not block and print_pose:
            while time.time() - start_t < duration:
                print("=======================")
                print(self.fa.get_joints())
                print(self.fa.get_pose())

    def calculate_tool_pose(self, joints):
        return self.fa.get_links_transforms(joints, use_rigid_transforms=True)[-2] * self.tool_to_base

    def get_tool_jacobian(self, joints):

        # Get the Jacobian of the end-effector (before tool attachment)
        J_ee = self.fa.get_jacobian(joints)

        # Base to tool transform
        base_to_tool = self.tool_to_base.inverse()
        R_tool, p_tool = base_to_tool.rotation, base_to_tool.translation

        # Skew-symmetric matrix for the translation part
        p_skew = np.array([
            [0, -p_tool[2], p_tool[1]],
            [p_tool[2], 0, -p_tool[0]],
            [-p_tool[1], p_tool[0], 0]
        ])

        # Construct the adjoint transformation matrix
        adj_T = np.block([
            [R_tool, np.zeros((3, 3))],
            [p_skew @ R_tool, R_tool]
        ])

        # Adjust the end-effector Jacobian to the tool frame
        tool_jacobian = adj_T @ J_ee
        return tool_jacobian

    def calculate_tool_velocity(self, joints):
        J_tool = self.get_tool_jacobian(joints)
        joint_velocities = self.fa.get_joint_velocities()
        return np.dot(J_tool, joint_velocities)

    def send_control_targets(self, pose_curr, actions, real_config, disassembly_path=None):
        """Sends pose targets to the running Cartesian-impedance controller via kukapy."""
        # NOTE: All actions are assumed to be in the form of [delta_position; delta_orientation],
        # where delta position is in the robot base frame, delta orientation is in the end-effector
        # frame, and delta orientation is an Euler vector (i.e., 3-element axis-angle
        # representation).

        curr_pos, curr_ori_mat = pose_curr.translation, pose_curr.rotation

        control_mode = real_config['control']['mode']['type']

        prop_gains = np.array(real_config['control']['prop_gains'], dtype=float)

        if disassembly_path is not None:
            disassembly_dir = disassembly_path[0] - disassembly_path[-1]
            disassembly_dir /= np.linalg.norm(disassembly_dir)
            arbitrary_dir = np.array([-1, 1, 1.0])
            arbitrary_dir /= np.linalg.norm(arbitrary_dir)
            x_prime = np.cross(arbitrary_dir, disassembly_dir)
            x_prime /= np.linalg.norm(x_prime)
            y_prime = np.cross(disassembly_dir, x_prime)
            rotation_matrix = np.column_stack((x_prime, y_prime, disassembly_dir))
            stiffness_matrix_disassembly = np.diag(prop_gains[:3])
            stiffness_matrix_world = rotation_matrix @ stiffness_matrix_disassembly @ rotation_matrix.T
            prop_gains[:3] = np.diag(stiffness_matrix_world)

        if control_mode == "nominal":
            targ_pos = curr_pos + actions[:3]
            targ_ori_mat = R.from_rotvec(actions[3:6]).as_matrix() @ curr_ori_mat

        elif control_mode in ["plai", "leaky_plai"]:
            if self._prev_targ_pos is None:
                self._prev_targ_pos = curr_pos.copy()
            if self._prev_targ_ori_mat is None:
                self._prev_targ_ori_mat = curr_ori_mat.copy()

            targ_pos = self._prev_targ_pos + actions[:3]
            targ_ori_mat = R.from_rotvec(actions[3:6]).as_matrix() @ self._prev_targ_ori_mat

            if control_mode == 'leaky_plai':
                pos_err = targ_pos - curr_pos
                pos_err_thresh = np.asarray(real_config['control']['mode']['leaky_plai']['pos_err_thresh'])
                if disassembly_path is not None:
                    thresh_matrix_disassembly = np.diag(pos_err_thresh)
                    thresh_matrix_world = rotation_matrix @ thresh_matrix_disassembly @ rotation_matrix.T
                    pos_err_thresh = np.diag(thresh_matrix_world)

                pos_err_clip = np.clip(
                    pos_err,
                    a_min=-pos_err_thresh,
                    a_max=pos_err_thresh,
                )
                targ_pos = curr_pos + pos_err_clip

            self._prev_targ_pos = targ_pos.copy()
            self._prev_targ_ori_mat = targ_ori_mat.copy()

        else:
            raise Exception(f"Invalid control mode")

        targ_pose = RigidTransform(
            translation=targ_pos, rotation=targ_ori_mat,
            from_frame=pose_curr.from_frame, to_frame=pose_curr.to_frame,
        )

        self.fa.publish_streaming_pose(targ_pose)

    def get_pose_error(self, pose_curr, pose_goal):
        curr_pos, curr_ori_mat = pose_curr.translation, pose_curr.rotation
        targ_pos, targ_ori_mat = pose_goal.translation, pose_goal.rotation
        pos_err = np.linalg.norm(targ_pos - curr_pos)
        ori_err_rad = (
            R.from_matrix(targ_ori_mat) * R.from_matrix(curr_ori_mat).inv()
        ).magnitude()
        return pos_err, ori_err_rad

    def get_rl_obs(self, pose_curr, pose_goal, disassembly_path=None):
        delta_pos = pose_goal.translation - pose_curr.translation
        if disassembly_path is not None:
            path_scale = np.linalg.norm(disassembly_path[0] - disassembly_path[-1]) / 0.02
            delta_pos = do_deltapos_path_transform(delta_pos, disassembly_path[-1], disassembly_path[0])
            delta_pos[2] /= path_scale
        obs = delta_pos
        obs = torch.as_tensor(obs, dtype=torch.float32, device=self.device)
        return obs

    def pose_world_to_robot_base(self, pos, quat):
        """Convert pose from world frame to robot base frame."""
        robot_base_pos = torch.tensor([[0.4540, 0.1975, 0.3950]], dtype=torch.float32, device=self.device)
        robot_base_quat = torch.tensor([[0.0, 0.0, -0.70314, 0.71105]], dtype=torch.float32, device=self.device)
        robot_base_transform_inv = torch_utils.tf_inverse(
            robot_base_quat, robot_base_pos
        )
        quat_in_robot_base, pos_in_robot_base = torch_utils.tf_combine(
            robot_base_transform_inv[0], robot_base_transform_inv[1], quat, pos
        )
        return pos_in_robot_base, quat_in_robot_base

    def get_rl_action(self, policy, real_config, obs, residual_action=None, disassembly_path=None):

        control_mode = real_config['control']['mode']['type']
        pos_action_scale = real_config['control']['mode'][control_mode]['pos_action_scale']

        policy.reset() # NOTE: RNN doesn't perform well somehow, have to clear states
        obs = policy.obs_to_torch(obs)
        action = policy.get_action(obs, is_determenistic=real_config['rl']['deterministic'])
        action = action.detach().cpu().numpy()

        if disassembly_path is not None:
            path_scale = np.linalg.norm(disassembly_path[0] - disassembly_path[-1]) / 0.02
            action[2] *= path_scale
            action = undo_deltapos_path_transform(action, disassembly_path[-1], disassembly_path[0])

        if residual_action is not None:
            if disassembly_path is not None:
                residual_action_transformed = do_deltapos_path_transform(residual_action, disassembly_path[-1], disassembly_path[0])
                residual_action_transformed[2] /= path_scale
                residual_action_norm = np.linalg.norm(residual_action_transformed)
            else:
                residual_action_norm = np.linalg.norm(residual_action)
            residual_action /= residual_action_norm
            action += residual_action

        action *= np.array(pos_action_scale)
        return action

    def execute_rl_policy(self, policy, real_config, goal_joints, global_pos_shift=np.zeros(3)):
        assert self.robot_num == 2

        duration = real_config['control']['duration']
        dt = 1.0 / real_config['control']['freq']

        curr_joints = self.fa.get_joints()
        pose_goal = self.calculate_tool_pose(goal_joints)
        pose_goal.translation += global_pos_shift
        pose_curr = self.calculate_tool_pose(curr_joints)

        disassembly_path = np.array([pose_goal.translation, pose_curr.translation])

        # plai
        self._prev_targ_pos = None
        self._prev_targ_ori_mat = None

        pos_err = None
        t_start = time.time()
        n_step = 0

        obs_noise = (2 * torch.rand(3).to(self.device) - 1) * 0.003

        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        tty.setcbreak(fd)

        success = False

        try:
            while time.time() - t_start < duration:

                key = get_key_nonblocking()
                if key == 'q':
                    print('Early stopped')
                    break

                curr_joints = self.fa.get_joints()
                pose_curr = self.calculate_tool_pose(curr_joints)

                residual_action = None
                if self.residual:
                    residual_action = disassembly_path[0][:3] - pose_curr.translation

                obs = self.get_rl_obs(pose_curr, pose_goal, disassembly_path)
                obs_raw = obs.detach().clone()
                if n_step % 10 == 0:
                    obs_noise = (2 * torch.rand(3).to(self.device) - 1) * 0.003
                obs += obs_noise
                action = self.get_rl_action(policy, real_config, obs, residual_action, disassembly_path)

                # print(f'step {n_step} | obs: {obs}')
                # print(f'step {n_step} | action: {action}')
                # print(f'step {n_step} | residual action: {residual_action}')
                # print(f'step {n_step} | joints: {self.fa.get_joints()}')

                action = np.concatenate([action, np.zeros(3)]) # zero delta orientation

                self.send_control_targets(pose_curr, action, real_config, disassembly_path)

                # If current pose is close enough to goal pose, terminate early
                pos_err, _ = self.get_pose_error(pose_curr, pose_goal)
                if obs_raw[-1].cpu().numpy() > -0.003:
                    print("Near goal, early termination")
                    success = True
                    break

                time.sleep(dt)
                n_step += 1

        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

        print(f'RL policy execution finished (pos err: {pos_err})')
        self.stop_skill()

        return success

    def get_openloop_action(self, real_config, pose_curr, pose_goal):

        control_mode = real_config['control']['mode']['type']
        pos_action_scale = real_config['control']['mode'][control_mode]['pos_action_scale']

        actions = pose_goal.translation - pose_curr.translation
        actions /= np.linalg.norm(actions)
        actions *= np.array(pos_action_scale)

        return actions

    def execute_openloop_policy(self, real_config, goal_joints):
        assert self.robot_num == 2

        duration = real_config['control']['duration']
        dt = 1.0 / real_config['control']['freq']

        curr_joints = self.fa.get_joints()
        pose_goal = self.calculate_tool_pose(goal_joints)
        pose_curr = self.calculate_tool_pose(curr_joints)

        disassembly_path = np.array([pose_goal.translation, pose_curr.translation])

        # plai
        self._prev_targ_pos = None
        self._prev_targ_ori_mat = None

        pos_err = None
        t_start = time.time()

        success = False

        try:
            while time.time() - t_start < duration:

                pose_curr = self.calculate_tool_pose(self.fa.get_joints())

                obs = self.get_rl_obs(pose_curr, pose_goal, disassembly_path) # only needed for success detection

                action = self.get_openloop_action(real_config, pose_curr, pose_goal)
                action = np.concatenate([action, np.zeros(3)]) # zero delta orientation

                self.send_control_targets(pose_curr, action, real_config, disassembly_path)

                # If current pose is close enough to goal pose, terminate early
                pos_err, _ = self.get_pose_error(pose_curr, pose_goal)
                if obs[-1].cpu().numpy() > -0.003:
                    print("Near goal, early termination")
                    success = True
                    break

                time.sleep(dt)

        except KeyboardInterrupt:
            print('Early stopped')

        print(f'Openloop policy execution finished (pos err: {pos_err})')
        self.stop_skill()

        return success
