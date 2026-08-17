from scipy.spatial.transform import Rotation as R
import numpy as np
from urdfpy import URDF
import pickle
import os
import yaml
import re


def modify_urdf_base(urdf_string, base_position, base_euler):
    """
    Modify the base position and orientation in a URDF string.

    Parameters:
        urdf_string: The original URDF content as a string.
        base_position: [x, y, z] position of the base.
        base_euler: [roll, pitch, yaw] orientation of the base in radians.

    Returns:
        modified_urdf: The updated URDF string.
    """
    import re

    # Convert base_position and base_euler to strings
    xyz_str = " ".join(map(str, base_position))
    rpy_str = " ".join(map(str, base_euler))

    # Find and modify the <origin> tag of the root link
    def replace_origin(match):
        return f'<origin xyz="{xyz_str}" rpy="{rpy_str}" />'

    modified_urdf = re.sub(
        r'<origin xyz="[^"]*" rpy="[^"]*"\s*/>',
        replace_origin,
        urdf_string,
        count=1  # Only modify the first <origin> tag
    )

    return modified_urdf


def calculate_relative_pose(object_pose, end_effector_pose):
    """
    Calculate the relative pose (rpy/xyz) of the object with respect to the end-effector.

    Parameters:
        object_pose: Tuple of (position, quaternion) of the object in the world frame.
        end_effector_pose: 4x4 homogeneous transformation matrix of the end-effector in the world frame.

    Returns:
        relative_xyz: [x, y, z] position of the object relative to the end-effector.
        relative_rpy: [roll, pitch, yaw] of the object relative to the end-effector.
    """
    object_position, object_quaternion = object_pose

    # Convert object's quaternion to a rotation matrix
    object_rotation = R.from_quat(object_quaternion).as_matrix()

    # Create homogeneous transformation for the object
    T_object = np.eye(4)
    T_object[:3, :3] = object_rotation
    T_object[:3, 3] = object_position

    # Compute the relative transformation matrix
    T_relative = np.linalg.inv(end_effector_pose) @ T_object

    # Extract relative position (xyz)
    relative_xyz = T_relative[:3, 3]

    # Extract relative rotation matrix and convert to RPY
    relative_rotation = T_relative[:3, :3]
    relative_rpy = R.from_matrix(relative_rotation).as_euler('xyz', degrees=False)

    return relative_xyz, relative_rpy


def calculate_fk(robot, joint_angles):
    # Create a mapping of joint names to their positions
    joint_positions = {}
    joint_names = [j.name for j in robot.actuated_joints]
    for name, angle in zip(joint_names, joint_angles):
        joint_positions[name] = angle

    # Compute FK to get the end-effector pose
    fk_transform = robot.link_fk(cfg=joint_positions)

    # Get the pose of the end-effector
    end_effector_link = "kuka_hand"  # Replace with the desired end-effector link name
    end_effector_pose = fk_transform[robot.link_map[end_effector_link]]

    return end_effector_pose


def inject_object_to_urdf(robot_urdf, object_urdf, parent_link_name, relative_xyz, relative_rpy):
    """
    Inject an object's URDF string into the robot URDF string as a child link.

    Parameters:
        robot_urdf: The original robot URDF string.
        object_urdf: The object URDF string to be injected.
        parent_link_name: The name of the parent link in the robot.
        relative_xyz: [x, y, z] position of the object relative to the parent link.
        relative_rpy: [roll, pitch, yaw] orientation of the object relative to the parent link.

    Returns:
        updated_urdf: The updated URDF string with the object injected.
    """
    # Parse the object URDF to extract its link definition
    object_link_match = re.search(r'<link.*?>.*?</link>', object_urdf, re.DOTALL)
    if not object_link_match:
        raise ValueError("Object URDF does not contain a valid <link> definition.")
    object_link = object_link_match.group()

    # Extract the object's link name
    object_link_name_match = re.search(r'name="(.*?)"', object_link)
    if not object_link_name_match:
        raise ValueError("Object URDF link does not have a name.")
    object_link_name = object_link_name_match.group(1)

    # Create a fixed joint to attach the object to the parent link
    object_joint = f"""
    <joint name="{object_link_name}_joint" type="fixed">
        <parent link="{parent_link_name}"/>
        <child link="{object_link_name}"/>
        <origin xyz="{' '.join(map(str, relative_xyz))}" rpy="{' '.join(map(str, relative_rpy))}" />
    </joint>
    """

    # Inject the object link and joint into the robot URDF
    updated_urdf = re.sub(
        r'(</robot>)',
        f'{object_link}\n{object_joint}\n\\1',  # Insert before the closing </robot> tag
        robot_urdf,
        flags=re.DOTALL
    )

    return updated_urdf


def update_finger_joints(urdf_string, finger_position):
    """
    Update the finger joints in the URDF string to be fixed at a specified position.

    Parameters:
        urdf_string: The original URDF content as a string.
        finger_position: The position for the gripper fingers.

    Returns:
        updated_urdf: The modified URDF string.
    """
    # Update kuka_finger_joint1
    def modify_joint1(match):
        return (
            f'<joint name="kuka_finger_joint1" type="fixed">\n'
            f'    <parent link="kuka_hand"/>\n'
            f'    <child link="kuka_leftfinger"/>\n'
            f'    <origin xyz="0.0 {finger_position} 0.0" rpy="0.0 0.0 0.0"/>\n'
            f'</joint>'
        )

    urdf_string = re.sub(
        r'<joint name="kuka_finger_joint1" .*?>.*?</joint>',
        modify_joint1,
        urdf_string,
        flags=re.DOTALL,
    )

    # Update kuka_finger_joint2
    def modify_joint2(match):
        return (
            f'<joint name="kuka_finger_joint2" type="fixed">\n'
            f'    <parent link="kuka_hand"/>\n'
            f'    <child link="kuka_rightfinger"/>\n'
            f'    <origin xyz="0.0 {-finger_position} 0.0" rpy="0.0 0.0 0.0"/>\n'
            f'</joint>'
        )

    updated_urdf = re.sub(
        r'<joint name="kuka_finger_joint2" .*?>.*?</joint>',
        modify_joint2,
        urdf_string,
        flags=re.DOTALL,
    )

    return updated_urdf


def load_assembly_plan_info(plan_info_dirname):
    """Load assembly plan info for plugs in each environment.

    Input:
    plan_info = {
        (part_plug, part_socket): {
            'arm_q_plug': [arm_q_plug_preassembly, arm_q_plug_assembled],
            'arm_q_socket': arm_q_socket,
            'open_ratio_plug': open_ratio_plug,
            'open_ratio_socket': open_ratio_socket,
            'path': path,
        },
        ...
    }
    Output:
    plan_info = {
        (part_plug, part_socket): {
            'arm_q': arm_q_plug_assembled,
            'open_ratio': open_ratio_plug,
        },
        ...
    }
    """
    plan_info_dir = os.path.join(os.path.dirname(__file__), '..', 'isaacgymenvs', 'tasks', 'fabrica', 'data', plan_info_dirname)
    plan_infos = {}
    for plan_file_name in os.listdir(plan_info_dir):
        plan_file_path = os.path.join(plan_info_dir, plan_file_name)
        if not plan_file_name.endswith('.pkl'):
            continue
        with open(plan_file_path, 'rb') as f:
            plan_info = pickle.load(f)
        plan_infos[plan_file_name.replace('.pkl', '')] = {
            (part_plug, part_socket): {
                'arm_q': plan_info[(part_plug, part_socket)]['arm_q_plug'][1],
                'open_ratio': plan_info[(part_plug, part_socket)]['open_ratio_plug'],
            }
            for (part_plug, part_socket) in plan_info
        }
    return plan_infos


def generate_kuka_urdf_from_plan(plan_info_dir, kuka_dir):
    """Generate URDF files for the KUKA robot with attached objects based on assembly plan info.

    `planning/` (planning/robot/util_arm.py, geometry.py) now has a native 'kuka' arm/gripper
    type (assets/kuka/kuka.urdf + meshes, ported from the KUKA rig's own description
    packages), so assemblies planned with `--arm kuka --gripper kuka` produce `arm_q` values
    that are natively KUKA joint angles, and this function's forward-kinematics step
    (using fabrica_kuka.urdf) is exact for them. Assemblies planned before this fork's KUKA
    support was added (arm_type/gripper_type == 'panda' in their grasps.pkl) still carry
    Panda-planned `arm_q`; running those through the KUKA chain here is only an approximation,
    since the two arms' link geometry isn't identical -- re-plan with `--arm kuka --gripper
    kuka` (or `planning/run_planning.sh EXP_NAME ASSEMBLY_NAME kuka`) for an exact result.
    """
    # Load the base URDF file for the KUKA robot
    base_robot_urdf_path = os.path.join(os.path.dirname(__file__), '..', 'assets', 'fabrica', 'urdf', 'fabrica_kuka.urdf')
    with open(base_robot_urdf_path, 'r') as f:
        base_robot_urdf = f.read()
    with open(base_robot_urdf_path, 'r') as f:
        robot = URDF.load(f)

    base_cfg_path = os.path.join(os.path.dirname(__file__), '..', 'isaacgymenvs', 'cfg', 'task', 'FabricaBase.yaml')
    with open(base_cfg_path, 'r') as f:
        base_cfg = yaml.safe_load(f)
    kuka_pos, kuka_euler = base_cfg['env']['kuka_pos'], base_cfg['env']['kuka_euler']
    kuka_pose = np.eye(4)
    kuka_pose[:3, 3] = kuka_pos
    kuka_pose[:3, :3] = R.from_euler('xyz', kuka_euler).as_matrix()
    assembly_center = base_cfg['env']['assembly_center']

    updated_urdf_dir = os.path.join(os.path.dirname(__file__), '..', 'assets', 'fabrica', 'urdf', kuka_dir)
    os.makedirs(updated_urdf_dir, exist_ok=True)

    plan_infos = load_assembly_plan_info(plan_info_dir)

    for assembly, plan_info in plan_infos.items():
        for part_plug, part_socket in plan_info:

            object_urdf_path = os.path.join(os.path.dirname(__file__), '..', 'assets', 'fabrica', 'urdf', 'fabrica', assembly, f'{part_plug}.urdf')
            with open(object_urdf_path, 'r') as f:
                base_object_urdf = f.read()
            updated_object_urdf = base_object_urdf.replace('filename="../', 'filename="')
            origin_tag = '<origin xyz="0 0 0" rpy="-1.5708 0 0"/>'
            updated_object_urdf = re.sub(
                r"(<visual>)",  # Match the opening <visual> tag and any whitespace after it
                r"<visual>\n\t" + origin_tag,  # Insert the origin tag and preserve indentation
                updated_object_urdf
            )

            arm_q, open_ratio = plan_info[(part_plug, part_socket)]['arm_q'], plan_info[(part_plug, part_socket)]['open_ratio']

            eef_pose = kuka_pose @ calculate_fk(robot, arm_q)
            relative_xyz, relative_rpy = calculate_relative_pose((assembly_center, [0, 0, 0, 1]), eef_pose)

            lower_limit, upper_limit = 0.0, 0.04
            open_ratio = min(open_ratio + 0.1, 1.0)
            finger_position = lower_limit + (upper_limit - lower_limit) * open_ratio

            updated_robot_urdf = base_robot_urdf.replace('filename="../', 'filename="../../')
            updated_robot_urdf = update_finger_joints(updated_robot_urdf, finger_position)
            updated_robot_urdf = inject_object_to_urdf(updated_robot_urdf, updated_object_urdf, "kuka_hand", relative_xyz, relative_rpy)
            updated_robot_urdf_path = os.path.join(updated_urdf_dir, f'{assembly}_{part_plug}_{part_socket}.urdf')
            with open(updated_robot_urdf_path, 'w') as f:
                f.write(updated_robot_urdf)


if __name__ == "__main__":
    from argparse import ArgumentParser
    parser = ArgumentParser()
    parser.add_argument('--plan-info-dir', type=str, default='plan_info')
    parser.add_argument('--kuka-dir', type=str, default='fabrica_kuka')
    args = parser.parse_args()

    generate_kuka_urdf_from_plan(args.plan_info_dir, args.kuka_dir)
    print("URDF generation completed.")
