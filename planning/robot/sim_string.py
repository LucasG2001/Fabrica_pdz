import os
import sys

project_base_dir = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..'))
sys.path.append(project_base_dir)

import numpy as np
import re
from planning.robot.geometry import get_ft_sensor_spec


def arr_to_str(arr):
    return ' '.join([str(x) for x in arr])


def get_color(index):
    colors = [
        [210, 87, 89, 255],
        [237, 204, 73, 255],
        [60, 167, 221, 255],
        [190, 126, 208, 255],
        [108, 192, 90, 255],
    ]
    colors = np.array(colors) / 255.0
    return colors[int(index) % 5]


def add_suffix(string, suffix):
    if suffix is None:
        return string
    else:
        return re.sub(r' name\s*=\s*"([^"]+)"', rf' name="\1_{suffix}"', string)


def get_panda_gripper_string(pos, quat, fixed):
    base_type = 'fixed' if fixed else 'free3d-exp'
    string = f'''
<robot>
    <link name="panda_hand">
        <joint name="panda_hand" type="{base_type}" pos="{arr_to_str(pos)}" quat="{arr_to_str(quat)}"/>
        <body name="panda_hand" type="mesh" filename="panda/visual/hand.obj" pos="0 0 0" quat="1 0 0 0" transform_type="OBJ_TO_JOINT" rgba = "1.0 1.0 1.0 1.0"/>
        <link name="panda_leftfinger">
            <joint name="panda_leftfinger" type="prismatic" axis="0 1 0" pos="0 0 5.84" quat="1 0 0 0" lim="0.0 4"/>
            <body name="panda_leftfinger" type="mesh" filename="panda/visual/finger.obj" pos="0 0 0" quat="1 0 0 0" transform_type="OBJ_TO_JOINT" rgba = "0.1 0.1 0.1 1.0"/>
        </link>
        <link name="panda_rightfinger">
            <joint name="panda_rightfinger" type="prismatic" axis="0 -1 0" pos="0 0 5.84" quat="1 0 0 0" lim="0.0 4"/>
            <body name="panda_rightfinger" type="mesh" filename="panda/visual/finger.obj" pos="0 0 0" quat="0 0 0 1" transform_type="OBJ_TO_JOINT" rgba = "0.1 0.1 0.1 1.0"/>
        </link>
    </link>
</robot>
'''
    return string


def get_kuka_gripper_string(pos, quat, fixed):
    # Same structure as get_panda_gripper_string, but finger joints mount directly at
    # kuka_hand's own origin (pos "0 0 0", not Panda's "0 0 5.84") -- matches
    # kuka_finger_joint1/2's origin in fabrica_kuka.urdf / kuka_iiwa7_y_gripper.urdf.
    base_type = 'fixed' if fixed else 'free3d-exp'
    string = f'''
<robot>
    <link name="kuka_hand">
        <joint name="kuka_hand" type="{base_type}" pos="{arr_to_str(pos)}" quat="{arr_to_str(quat)}"/>
        <body name="kuka_hand" type="mesh" filename="kuka/visual/hand.obj" pos="0 0 0" quat="1 0 0 0" transform_type="OBJ_TO_JOINT" rgba = "1.0 1.0 1.0 1.0"/>
        <link name="kuka_leftfinger">
            <joint name="kuka_leftfinger" type="prismatic" axis="0 1 0" pos="0 0 0" quat="1 0 0 0" lim="0.0 4"/>
            <body name="kuka_leftfinger" type="mesh" filename="kuka/visual/left_finger.obj" pos="0 0 0" quat="1 0 0 0" transform_type="OBJ_TO_JOINT" rgba = "0.1 0.1 0.1 1.0"/>
        </link>
        <link name="kuka_rightfinger">
            <joint name="kuka_rightfinger" type="prismatic" axis="0 -1 0" pos="0 0 0" quat="1 0 0 0" lim="0.0 4"/>
            <body name="kuka_rightfinger" type="mesh" filename="kuka/visual/right_finger.obj" pos="0 0 0" quat="1 0 0 0" transform_type="OBJ_TO_JOINT" rgba = "0.1 0.1 0.1 1.0"/>
        </link>
    </link>
</robot>
'''
    return string


def get_pdz_gripper_string(gripper_type, pos, quat, fixed):
    """Build the RedMax visualization model for a PDZ gripper variant.

    PDZ meshes are authored in the flange frame at the fully closed pose. The
    two fingers therefore only need their signed prismatic displacement. The
    complete variants also carry the separately authored D405 collision box so
    that the recorded motion depicts the same occupied volume used by planning.
    """
    base_type = 'fixed' if fixed else 'free3d-exp'
    folder = gripper_type.replace('-', '_')
    d405_link = ''
    if '-mech' not in gripper_type:
        d405_link = f'''
        <link name="pdz_d405">
            <joint name="pdz_d405" type="fixed" pos="0 0 0" quat="1 0 0 0"/>
            <body name="pdz_d405" type="mesh" filename="{folder}/collision/d405.obj" pos="0 0 0" quat="1 0 0 0" transform_type="OBJ_TO_JOINT" rgba="0.12 0.12 0.12 1.0"/>
        </link>'''
    string = f'''
<robot>
    <link name="pdz_base">
        <joint name="pdz_base" type="{base_type}" pos="{arr_to_str(pos)}" quat="{arr_to_str(quat)}"/>
        <body name="pdz_base" type="mesh" filename="{folder}/visual/base.obj" pos="0 0 0" quat="1 0 0 0" transform_type="OBJ_TO_JOINT" rgba="0.28 0.28 0.30 1.0"/>
        <link name="pdz_left_finger">
            <joint name="pdz_left_finger" type="prismatic" axis="-1 0 0" pos="0 0 0" quat="1 0 0 0" lim="0.0 3.2"/>
            <body name="pdz_left_finger" type="mesh" filename="{folder}/visual/finger_left.obj" pos="0 0 0" quat="1 0 0 0" transform_type="OBJ_TO_JOINT" rgba="0.12 0.12 0.12 1.0"/>
        </link>
        <link name="pdz_right_finger">
            <joint name="pdz_right_finger" type="prismatic" axis="1 0 0" pos="0 0 0" quat="1 0 0 0" lim="0.0 3.2"/>
            <body name="pdz_right_finger" type="mesh" filename="{folder}/visual/finger_right.obj" pos="0 0 0" quat="1 0 0 0" transform_type="OBJ_TO_JOINT" rgba="0.12 0.12 0.12 1.0"/>
        </link>{d405_link}
    </link>
</robot>
'''
    return string


def get_robotiq_85_gripper_string(pos, quat, fixed):
    base_type = 'fixed' if fixed else 'free3d-exp'
    string = f'''
<robot>
    <link name="robotiq_base">
        <joint name="robotiq_base" type="{base_type}" pos="{arr_to_str(pos)}" quat="{arr_to_str(quat)}"/>
        <body name= "robotiq_base" type = "mesh" filename = "robotiq_85/visual/robotiq_base_fine.obj" pos = "0 0 0" quat = "1 0 0 0" transform_type="OBJ_TO_JOINT" rgba = "0.1 0.1 0.1 1.0"/>
        <link name="robotiq_left_outer_knuckle">
            <joint name = "robotiq_left_outer_knuckle" type="revolute" pos="3.06011444260539 0.0 6.27920162695395" quat="1.0 0.0 0.0 0.0" axis="0.0 -1.0 0.0" lim="0.0 0.8757"/>
            <body name= "robotiq_left_outer_knuckle" type = "mesh" filename = "robotiq_85/visual/outer_knuckle_fine.obj" pos = "0 0 0" quat = "1 0 0 0" transform_type="OBJ_TO_JOINT" rgba = "0.1 0.1 0.1 1.0"/>
            <link name="robotiq_left_outer_finger">
                <joint name="robotiq_left_outer_finger" type="fixed" pos="3.16910442266543 0.0 -0.193396375724605" quat="1.0 0.0 0.0 0.0"/>
                <body name= "robotiq_left_outer_finger" type = "mesh" filename = "robotiq_85/visual/outer_finger_fine.obj" pos = "0 0 0" quat = "1 0 0 0" transform_type="OBJ_TO_JOINT" rgba = "0.1 0.1 0.1 1.0"/>
            </link>
        </link>
        <link name="robotiq_left_inner_knuckle">
            <joint name = "robotiq_left_inner_knuckle" type="revolute" pos="1.27000000001501 0.0 6.93074999999639" quat="1.0 0.0 0.0 0.0" axis="0.0 -1.0 0.0" lim="0.0 0.8757"/>
            <body name= "robotiq_left_inner_knuckle" type = "mesh" filename = "robotiq_85/visual/inner_knuckle_fine.obj" pos = "0 0 0" quat = "1 0 0 0" transform_type="OBJ_TO_JOINT" rgba = "0.1 0.1 0.1 1.0"/>
            <link name="robotiq_left_inner_finger">
                <joint name = "robotiq_left_inner_finger" type="revolute" pos="3.4585310861294003 0.0 4.5497019381797505" quat="1.0 0.0 0.0 0.0" axis="0.0 -1.0 0.0" lim="-0.8757 0.0"/>
                <body name= "robotiq_left_inner_finger" type = "mesh" filename = "robotiq_85/visual/inner_finger_fine.obj" pos = "0 0 0" quat = "1 0 0 0" transform_type="OBJ_TO_JOINT" rgba = "0.1 0.1 0.1 1.0"/>
            </link>
        </link>
        <link name="robotiq_right_outer_knuckle">
            <joint name = "robotiq_right_outer_knuckle" type="revolute" pos="-3.06011444260539 0.0 6.27920162695395" quat="0.0 0.0 0.0 1.0" axis="0.0 -1.0 0.0" lim="0.0 0.8757"/>
            <body name= "robotiq_right_outer_knuckle" type = "mesh" filename = "robotiq_85/visual/outer_knuckle_fine.obj" pos = "0 0 0" quat = "1 0 0 0" transform_type="OBJ_TO_JOINT" rgba = "0.1 0.1 0.1 1.0"/>
            <link name="robotiq_right_outer_finger">
                <joint name="robotiq_right_outer_finger" type="fixed" pos="3.16910442266543 0.0 -0.193396375724605" quat="1.0 0.0 0.0 0.0"/>
                <body name= "robotiq_right_outer_finger" type = "mesh" filename = "robotiq_85/visual/outer_finger_fine.obj" pos = "0 0 0" quat = "1 0 0 0" transform_type="OBJ_TO_JOINT" rgba = "0.1 0.1 0.1 1.0"/>
            </link>
        </link>
        <link name="robotiq_right_inner_knuckle">
            <joint name = "robotiq_right_inner_knuckle" type="revolute" pos="-1.27000000001501 0.0 6.93074999999639" quat="0.0 0.0 0.0 1.0" axis="0.0 1.0 0.0" lim="-0.8757 0.0"/>
            <body name= "robotiq_right_inner_knuckle" type = "mesh" filename = "robotiq_85/visual/inner_knuckle_fine.obj" pos = "0 0 0" quat = "1 0 0 0" transform_type="OBJ_TO_JOINT" rgba = "0.1 0.1 0.1 1.0"/>
            <link name="robotiq_right_inner_finger">
                <joint name = "robotiq_right_inner_finger" type="revolute" pos="3.4585310861294003 0.0 4.5497019381797505" quat="1.0 0.0 0.0 0.0" axis="0.0 1.0 0.0" lim="0.0 0.8757" damping="0.0"/>
                <body name= "robotiq_right_inner_finger" type = "mesh" filename = "robotiq_85/visual/inner_finger_fine.obj" pos = "0 0 0" quat = "1 0 0 0" transform_type="OBJ_TO_JOINT" rgba = "0.1 0.1 0.1 1.0"/>
            </link>
        </link>
    </link>
</robot>
'''
    return string


def get_robotiq_140_gripper_string(pos, quat, fixed):
    base_type = 'fixed' if fixed else 'free3d-exp'
    string = f'''
<robot>
    <link name="robotiq_base">
        <joint name="robotiq_base" type="{base_type}" pos="{arr_to_str(pos)}" quat="{arr_to_str(quat)}"/>
        <body name= "robotiq_base" type = "mesh" filename = "robotiq_140/visual/robotiq_base_fine.obj" pos = "0 0 0" quat = "1 0 0 0" transform_type="OBJ_TO_JOINT" rgba = "0.1 0.1 0.1 1.0"/>
        <link name="robotiq_left_outer_knuckle">
            <joint name = "robotiq_left_outer_knuckle" type="revolute" pos="0 -3.0601 5.4905" quat="0.41040502 0.91190335 0.0 0.0" axis="-1.0 0.0 0.0" lim="0.0 0.8757"/>
            <body name= "robotiq_left_outer_knuckle" type = "mesh" filename = "robotiq_140/visual/outer_knuckle_fine.obj" pos = "0 0 0" quat = "1 0 0 0" transform_type="OBJ_TO_JOINT" rgba = "0.1 0.1 0.1 1.0"/>
            <link name="robotiq_left_outer_finger">
                <joint name = "robotiq_left_outer_finger" type="fixed" pos="0 1.821998610742 2.60018192872234" quat="1.0 0.0 0.0 0.0" axis="1.0 0.0 0.0"/>
                <body name= "robotiq_left_outer_finger" type = "mesh" filename = "robotiq_140/visual/outer_finger_fine.obj" pos = "0 0 0" quat = "1 0 0 0" transform_type="OBJ_TO_JOINT" rgba = "0.1 0.1 0.1 1.0"/>
                <link name="robotiq_left_inner_finger">
                    <joint name = "robotiq_left_inner_finger" type="revolute" pos="0 8.17554015893473 -2.82203446692936" quat="0.93501321 -0.35461287 0.0 0.0" axis="1.0 0.0 0.0"/>
                    <body name= "robotiq_left_inner_finger" type = "mesh" filename = "robotiq_140/visual/inner_finger_fine.obj" pos = "0 0 0" quat = "1 0 0 0" transform_type="OBJ_TO_JOINT" rgba = "0.1 0.1 0.1 1.0"/>
                    <link name="robotiq_left_pad">
                        <joint name = "robotiq_left_pad" type="fixed" pos="0 3.8 -2.3" quat="0.0 0.0 0.70710678 0.70710678" axis="1.0 0.0 0.0"/>
                        <body name= "robotiq_left_pad" type = "mesh" filename = "robotiq_140/visual/pad_fine.obj" pos = "0 0 0" quat = "1 0 0 0" transform_type="OBJ_TO_JOINT" rgba = "0.1 0.1 0.1 1.0"/>
                    </link>
                </link>
            </link>
        </link>
        <link name="robotiq_left_inner_knuckle">
            <joint name = "robotiq_left_inner_knuckle" type="revolute" pos="0 -1.27 6.142" quat="0.41040502 0.91190335 0.0 0.0" axis="1.0 0.0 0.0" lim="0.0 0.8757"/>
            <body name= "robotiq_left_inner_knuckle" type = "mesh" filename = "robotiq_140/visual/inner_knuckle_fine.obj" pos = "0 0 0" quat = "1 0 0 0" transform_type="OBJ_TO_JOINT" rgba = "0.1 0.1 0.1 1.0"/>
        </link>
        <link name="robotiq_right_outer_knuckle">
            <joint name = "robotiq_right_outer_knuckle" type="revolute" pos="0 3.0601 5.4905" quat="0.0 0.0 0.91190335 0.41040502" axis="1.0 0.0 0.0" lim="0.0 0.8757"/>
            <body name= "robotiq_right_outer_knuckle" type = "mesh" filename = "robotiq_140/visual/outer_knuckle_fine.obj" pos = "0 0 0" quat = "1 0 0 0" transform_type="OBJ_TO_JOINT" rgba = "0.1 0.1 0.1 1.0"/>
            <link name="robotiq_right_outer_knuckle">
                <joint name = "robotiq_right_outer_finger" type="fixed" pos="0 1.821998610742 2.60018192872234" quat="1.0 0.0 0.0 0.0" axis="1.0 0.0 0.0"/>
                <body name= "robotiq_right_outer_finger" type = "mesh" filename = "robotiq_140/visual/outer_finger_fine.obj" pos = "0 0 0" quat = "1 0 0 0" transform_type="OBJ_TO_JOINT" rgba = "0.1 0.1 0.1 1.0"/>
                <link name="robotiq_right_inner_finger">
                    <joint name = "robotiq_right_inner_finger" type="revolute" pos="0 8.17554015893473 -2.82203446692936" quat="0.93501321 -0.35461287 0.0 0.0" axis="1.0 0.0 0.0"/>
                    <body name= "robotiq_right_inner_finger" type = "mesh" filename = "robotiq_140/visual/inner_finger_fine.obj" pos = "0 0 0" quat = "1 0 0 0" transform_type="OBJ_TO_JOINT" rgba = "0.1 0.1 0.1 1.0"/>
                    <link name="robotiq_right_pad">
                        <joint name = "robotiq_right_pad" type="fixed" pos="0 3.8 -2.3" quat="0.0 0.0 0.70710678 0.70710678" axis="1.0 0.0 0.0"/>
                        <body name= "robotiq_right_pad" type = "mesh" filename = "robotiq_140/visual/pad_fine.obj" pos = "0 0 0" quat = "1 0 0 0" transform_type="OBJ_TO_JOINT" rgba = "0.1 0.1 0.1 1.0"/>
                    </link>
                </link>
            </link>
        </link>
        <link name="robotiq_right_inner_knuckle">
            <joint name = "robotiq_right_inner_knuckle" type="revolute" pos="0 1.27 6.142" quat="0.0 0.0 -0.91190335 -0.41040502" axis="1.0 0.0 0.0" lim="0.0 0.8757"/>
            <body name= "robotiq_right_inner_knuckle" type = "mesh" filename = "robotiq_140/visual/inner_knuckle_fine.obj" pos = "0 0 0" quat = "1 0 0 0" transform_type="OBJ_TO_JOINT" rgba = "0.1 0.1 0.1 1.0"/>
        </link>
    </link>
</robot>
'''
    return string


def get_ft_sensor_substring():
    ft_spec = get_ft_sensor_spec()
    string = f'''
    <link name="ft_sensor">
        <joint name = "ft_sensor" type="fixed" pos="0 0 {-ft_spec['height'] / 2}" quat="1 0 0 0"/>
        <body name= "ft_sensor" type = "cylinder" pos = "0 0 0" quat = "1 0 0 0" radius = "{ft_spec['radius']}" height = "{ft_spec['height']}" transform_type="OBJ_TO_JOINT" gravity="false" rgba = "0.2 0.2 0.2 1.0"/>
    </link>
    '''
    return string


def get_gripper_string(gripper_type, pos, quat, fixed, has_ft_sensor=False, suffix=None):
    if gripper_type == 'panda':
        string = get_panda_gripper_string(pos, quat, fixed)
    elif gripper_type == 'kuka':
        string = get_kuka_gripper_string(pos, quat, fixed)
    elif gripper_type == 'robotiq-85':
        string = get_robotiq_85_gripper_string(pos, quat, fixed)
    elif gripper_type == 'robotiq-140':
        string = get_robotiq_140_gripper_string(pos, quat, fixed)
    elif gripper_type.startswith('pdz'):
        string = get_pdz_gripper_string(gripper_type, pos, quat, fixed)
    else:
        raise ValueError('Unknown gripper type: {}'.format(gripper_type))
    
    if has_ft_sensor: # insert ft sensor string after the second-last </link> tag
        link_closing_tags = [m.start() for m in re.finditer(r"</link>", string)]
        assert len(link_closing_tags) >= 2
        second_last_pos = link_closing_tags[-2]
        string = string[:second_last_pos + len("</link>")] + get_ft_sensor_substring() + string[second_last_pos + len("</link>"):]
        
    return add_suffix(string, suffix)


def get_xarm7_arm_string(pos, quat):
    string = f'''
<robot>
    <link name="linkbase">
        <joint name="linkbase" type="fixed" pos="{arr_to_str(pos)}" quat="{arr_to_str(quat)}"/>
        <body name= "linkbase" type = "abstract" pos = "-2.1131 -0.16302 5.6488" quat = "0.41928822390350623 -0.3384325829202692 0.5661965601674424 0.6237645608467303" mass = "885.5600000000001" inertia = "16772.46795287213 33528.24905872843 38202.28298839946" rgba = "0.8 0.8 0.8 1.0">
            <visual mesh = "xarm7/visual/linkbase_smooth.obj" pos = "4.2037215880446706 -4.303182668106523 -0.4604913739852395" quat = "-0.4192882239035062 -0.3384325829202692 0.5661965601674422 0.6237645608467302"/>
            <collision contacts = "xarm7/contacts/linkbase.txt" pos = "4.2037215880446706 -4.303182668106523 -0.4604913739852395" quat = "-0.4192882239035062 -0.3384325829202692 0.5661965601674422 0.6237645608467302"/>
        </body>
        <link name="joint1">
            <joint name = "joint1" type="revolute" pos="0.0 0.0 26.700000000000003" quat="1.0 0.0 0.0 0.0" axis="0.0 0.0 1.0" lim="-6.28318530718 6.28318530718" damping="0.0"/>
            <body name= "link1" type = "abstract" pos = "-0.42142 2.8209999999999997 -0.87788" quat = "0.6918679191340069 0.0005252148724432689 -0.6060703296327886 0.39242484906198305" mass = "426.03000000000003" inertia = "8235.113114059062 13775.66035044562 14455.126535495323" rgba = "0.9 0.9 0.9 1.0">
                <visual mesh = "xarm7/visual/link1_smooth.obj" pos = "-0.8114216775604369 -2.5981972214308087 1.2236319587744608" quat = "0.6918679191340068 -0.0005252148724432888 0.6060703296327885 -0.392424849061983"/>
                <collision contacts = "xarm7/contacts/link1_vhacd.txt" pos = "-0.8114216775604369 -2.5981972214308087 1.2236319587744608" quat = "0.6918679191340068 -0.0005252148724432888 0.6060703296327885 -0.392424849061983"/>
            </body>
            <link name="joint2">
                <joint name = "joint2" type="revolute" pos="0.0 0.0 0.0" quat="0.7071054825112364 -0.7071080798594735 0.0 0.0" axis="0.0 0.0 1.0" lim="-2.059 2.0944" damping="0.0"/>
                <body name= "link2" type = "abstract" pos = "-0.0033178 -12.849 2.6337" quat = "0.6337905179370223 -0.3150527964362565 0.6307030926183779 -0.3182215011472822" mass = "560.9499999999999" inertia = "9808.04401005623 31159.849558318598 31915.106431625172" rgba = "0.8 0.8 0.8 1.0">
                    <visual mesh = "xarm7/visual/link2_smooth.obj" pos = "-8.711764384019158 9.804940501796535 -0.03861050843999272" quat = "0.6337905179370222 0.3150527964362564 -0.6307030926183778 0.31822150114728215"/>
                    <collision contacts = "xarm7/contacts/link2_vhacd.txt" pos = "-8.711764384019158 9.804940501796535 -0.03861050843999272" quat = "0.6337905179370222 0.3150527964362564 -0.6307030926183778 0.31822150114728215"/>
                </body>
                <link name="joint3">
                    <joint name = "joint3" type="revolute" pos="0.0 -29.299999999999997 0.0" quat="0.7071054825112364 0.7071080798594735 0.0 0.0" axis="0.0 0.0 1.0" lim="-6.28318530718 6.28318530718" damping="0.0"/>
                    <body name= "link3" type = "abstract" pos = "4.223 -2.3258 -0.9667399999999999" quat = "-0.24066027491433598 0.8530839593824141 -0.23989343952005776 0.39595647235247505" mass = "444.63000000000005" inertia = "7804.745076690154 11912.598616264928 13322.65630704492" rgba = "0.9 0.9 0.9 1.0">
                        <visual mesh = "xarm7/visual/link3_smooth.obj" pos = "-3.266493961637458 -1.4456637070979548 -3.379013837226153" quat = "0.24066027491433598 0.8530839593824141 -0.23989343952005787 0.39595647235247505"/>
                        <collision contacts = "xarm7/contacts/link3_vhacd.txt" pos = "-3.266493961637458 -1.4456637070979548 -3.379013837226153" quat = "0.24066027491433598 0.8530839593824141 -0.23989343952005787 0.39595647235247505"/>
                    </body>
                    <link name="joint4">
                        <joint name = "joint4" type="revolute" pos="5.25 0.0 0.0" quat="0.7071054825112364 0.7071080798594735 0.0 0.0" axis="0.0 0.0 1.0" lim="-0.19198 3.927" damping="0.0"/>
                        <body name= "link4" type = "abstract" pos = "6.7148 -10.732 2.4479" quat = "0.6707722696010192 0.47605887893032944 0.012741086884645855 -0.5685685278230704" mass = "523.8699999999999" inertia = "8944.094777271237 28270.539922454394 28898.36530027436" rgba = "0.8 0.8 0.8 1.0">
                            <visual mesh = "xarm7/visual/link4_smooth.obj" pos = "-9.059983366567941 -7.802235142560355 -4.826842200415131" quat = "0.6707722696010191 -0.47605887893032933 -0.012741086884645859 0.5685685278230704"/>
                            <collision contacts = "xarm7/contacts/link4_vhacd.txt" pos = "-9.059983366567941 -7.802235142560355 -4.826842200415131" quat = "0.6707722696010191 -0.47605887893032933 -0.012741086884645859 0.5685685278230704"/>
                        </body>
                        <link name="joint5">
                            <joint name = "joint5" type="revolute" pos="7.75 -34.25 0.0" quat="0.7071054825112364 0.7071080798594735 0.0 0.0" axis="0.0 0.0 1.0" lim="-6.28318530718 6.28318530718" damping="0.0"/>
                            <body name= "link5" type = "abstract" pos = "-0.023397 3.6705 -8.0064" quat = "0.6892055355098681 -0.16046408568558085 0.698228216539248 -0.10827910535309972" mass = "185.54000000000002" inertia = "2471.2608713476575 9886.134859618787 9955.304269033553" rgba = "0.9 0.9 0.9 1.0">
                                <visual mesh = "xarm7/visual/link5_smooth.obj" pos = "-6.057144261834268 -6.378684331187784 -0.4460361240938076" quat = "-0.6892055355098679 -0.16046408568558085 0.6982282165392479 -0.1082791053530997"/>
                                <collision contacts = "xarm7/contacts/link5_vhacd.txt" pos = "-6.057144261834268 -6.378684331187784 -0.4460361240938076" quat = "-0.6892055355098679 -0.16046408568558085 0.6982282165392479 -0.1082791053530997"/>
                            </body>
                            <link name="joint6">
                                <joint name = "joint6" type="revolute" pos="0.0 0.0 0.0" quat="0.7071054825112364 0.7071080798594735 0.0 0.0" axis="0.0 0.0 1.0" lim="-1.69297 3.14159265359" damping="0.0"/>
                                <body name= "link6" type = "abstract" pos = "5.8911 2.8469 0.68428" quat = "0.9529732922502667 0.01599291734637919 0.1692539717525629 0.250876909855057" mass = "313.44" inertia = "3867.077886736355 7688.706404074782 8278.915709188861" rgba = "1.0 1.0 1.0 1.0">
                                    <visual mesh = "xarm7/visual/link6_smooth.obj" pos = "-5.973444004856806 0.21893372810568068 -2.747393798118139" quat = "0.9529732922502666 -0.01599291734637919 -0.16925397175256282 -0.250876909855057"/>
                                    <collision contacts = "xarm7/contacts/link6_vhacd.txt" pos = "-5.973444004856806 0.21893372810568068 -2.747393798118139" quat = "0.9529732922502666 -0.01599291734637919 -0.16925397175256282 -0.250876909855057"/>
                                </body>
                                <link name="joint7">
                                    <joint name = "joint7" type="revolute" pos="7.6 9.700000000000001 0.0" quat="0.7071054825112364 -0.7071080798594735 0.0 0.0" axis="0.0 0.0 1.0" lim="-6.28318530718 6.28318530718" damping="0.0"/>
                                    <body name= "link7" type = "abstract" pos = "-0.0015846 -0.46376999999999996 -1.2705" quat = "-0.0051369063433354505 0.7078680662792562 -0.706304495783441 -0.005511095298183751" mass = "314.68" inertia = "1192.0774998754055 1698.502197488278 2603.520302636317" rgba = "0.753 0.753 0.753 1.0">
                                        <visual mesh = "xarm7/visual/link7_smooth.obj" pos = "-0.4828448608210097 -0.0019607575065535687 -1.2633734086428683" quat = "0.0051369063433354505 0.7078680662792562 -0.706304495783441 -0.005511095298183751"/>
                                        <collision contacts = "xarm7/contacts/link7_vhacd.txt" pos = "-0.4828448608210097 -0.0019607575065535687 -1.2633734086428683" quat = "0.0051369063433354505 0.7078680662792562 -0.706304495783441 -0.005511095298183751"/>
                                    </body>
                                </link>
                            </link>
                        </link>
                    </link>
                </link>
            </link>
        </link>
    </link>
</robot>
'''
    return string


def get_panda_arm_string(pos, quat):
    string = f'''
<robot>
    <link name="panda_link0">
        <joint name="panda_joint0" type="fixed" pos="{arr_to_str(pos)}" quat="{arr_to_str(quat)}"/>
        <body name="panda_link0" type="abstract" pos="0 0 0" quat="1 0 0 0" mass="2700" inertia="10000 0 0 10000 0 10000" rgba="0.8 0.8 0.8 1.0">
            <visual mesh="panda/visual/link0.obj" pos="0 0 0" quat="1 0 0 0"/>
            <collision contacts="panda/contacts/link0.txt" pos="0 0 0" quat="1 0 0 0"/>
        </body>
        <link name="panda_link1">
            <joint name="panda_joint1" type="revolute" axis="0 0 1" pos="0 0 33.3" quat="1 0 0 0" lim="-2.9671 2.9671"/>
            <body name="panda_link1" type="abstract" pos="0 0 0" quat="1 0 0 0" mass="2700" inertia="10000 0 0 10000 0 10000" rgba="0.9 0.9 0.9 1.0">
                <visual mesh="panda/visual/link1.obj" pos="0 0 0" quat="1 0 0 0"/>
                <collision contacts="panda/contacts/link1.txt" pos="0 0 0" quat="1 0 0 0"/>
            </body>
            <link name="panda_link2">
                <joint name="panda_joint2" type="revolute" axis="0 0 1" pos="0 0 0" quat="-0.7071068 0.7071068 0 0" lim="-1.8326 1.8326"/>
                <body name="panda_link2" type="abstract" pos="0 0 0" quat="1 0 0 0" mass="2730" inertia="10000 0 0 10000 0 10000" rgba="0.8 0.8 0.8 1.0">
                    <visual mesh="panda/visual/link2.obj" pos="0 0 0" quat="1 0 0 0"/>
                    <collision contacts="panda/contacts/link2.txt" pos="0 0 0" quat="1 0 0 0"/>
                </body>
                <link name="panda_link3">
                    <joint name="panda_joint3" type="revolute" axis="0 0 1" pos="0 -31.6 0" quat="0.7071068 0.7071068 0 0" lim="-2.9671 2.9671"/>
                    <body name="panda_link3" type="abstract" pos="0 0 0" quat="1 0 0 0" mass="2040" inertia="10000 0 0 10000 0 10000" rgba="0.9 0.9 0.9 1.0">
                        <visual mesh="panda/visual/link3.obj" pos="0 0 0" quat="1 0 0 0"/>
                        <collision contacts="panda/contacts/link3.txt" pos="0 0 0" quat="1 0 0 0"/>
                    </body>
                    <link name="panda_link4">
                        <joint name="panda_joint4" type="revolute" axis="0 0 1" pos="8.25 0 0" quat="0.7071068 0.7071068 0 0" lim="-3.1416 3.1416"/>
                        <body name="panda_link4" type="abstract" pos="0 0 0" quat="1 0 0 0" mass="2080" inertia="10000 0 0 10000 0 10000" rgba="0.8 0.8 0.8 1.0">
                            <visual mesh="panda/visual/link4.obj" pos="0 0 0" quat="1 0 0 0"/>
                            <collision contacts="panda/contacts/link4.txt" pos="0 0 0" quat="1 0 0 0"/>
                        </body>
                        <link name="panda_link5">
                            <joint name="panda_joint5" type="revolute" axis="0 0 1" pos="-8.25 38.4 0" quat="-0.7071068 0.7071068 0 0" lim="-2.9671 2.9671"/>
                            <body name="panda_link5" type="abstract" pos="0 0 0" quat="1 0 0 0" mass="3000" inertia="10000 0 0 10000 0 10000" rgba="0.9 0.9 0.9 1.0">
                                <visual mesh="panda/visual/link5.obj" pos="0 0 0" quat="1 0 0 0"/>
                                <collision contacts="panda/contacts/link5.txt" pos="0 0 0" quat="1 0 0 0"/>
                            </body>
                            <link name="panda_link6">
                                <joint name="panda_joint6" type="revolute" axis="0 0 1" pos="0 0 0" quat="0.7071068 0.7071068 0 0" lim="-0.0873 3.8223"/>
                                <body name="panda_link6" type="abstract" pos="0 0 0" quat="1 0 0 0" mass="1300" inertia="10000 0 0 10000 0 10000" rgba="0.8 0.8 0.8 1.0">
                                    <visual mesh="panda/visual/link6.obj" pos="0 0 0" quat="1 0 0 0"/>
                                    <collision contacts="panda/contacts/link6.txt" pos="0 0 0" quat="1 0 0 0"/>
                                </body>
                                <link name="panda_link7">
                                    <joint name="panda_joint7" type="revolute" axis="0 0 1" pos="8.8 0 0" quat="0.7071068 0.7071068 0 0" lim="-2.9671 2.9671"/>
                                    <body name="panda_link7" type="abstract" pos="0 0 0" quat="1 0 0 0" mass="200" inertia="10000 0 0 10000 0 10000" rgba="0.9 0.9 0.9 1.0">
                                        <visual mesh="panda/visual/link7.obj" pos="0 0 0" quat="1 0 0 0"/>
                                        <collision contacts="panda/contacts/link7.txt" pos="0 0 0" quat="1 0 0 0"/>
                                    </body>
                                    <link name="panda_link8">
                                        <joint name="panda_joint8" type="fixed" pos="0 0 10.7" quat="1 0 0 0"/>
                                        <body name="panda_link8" type="sphere" radius="0.01" pos="0 0 0" quat="1 0 0 0" rgba="0.9 0.9 0.9 1.0"/>
                                    </link>
                                </link>
                            </link>
                        </link>
                    </link>
                </link>
            </link>
        </link>
    </link>
</robot>
'''
    return string


def get_kuka_arm_string(pos, quat):
    # Same "abstract body + mesh visual + point-cloud contacts" structure as
    # get_panda_arm_string, but joint axes are NOT all forced to "0 0 1": unlike Panda's URDF
    # (which alternates the per-joint origin *rotation* so every joint's own axis stays local
    # Z), fabrica_kuka.urdf / assets/kuka/kuka.urdf keep joint origins unrotated and vary the
    # `axis` attribute instead (0 0 1 for joints 1/3/5/7, 0 +-1 0 for joints 2/4/6) -- ported
    # directly from kuka_iiwa7_y_gripper.urdf's own convention. redmax's JointRevolute takes an
    # arbitrary axis vector (Simulation_Constructor.cpp), so this is a direct, exact port of
    # the same joint pos/axis/limit values already validated in fabrica_kuka.urdf and
    # assets/kuka/kuka.urdf -- no axis-remapping/quat-conversion was needed. Mass values are
    # placeholders copied from get_panda_arm_string's own (already non-physical, uniform-density
    # "abstract" body) values -- same as upstream, since this feeds rendering/playback only
    # (not the RL training or motion-planning physics).
    string = f'''
<robot>
    <link name="kuka_link0">
        <joint name="kuka_joint0" type="fixed" pos="{arr_to_str(pos)}" quat="{arr_to_str(quat)}"/>
        <body name="kuka_link0" type="abstract" pos="0 0 0" quat="1 0 0 0" mass="2700" inertia="10000 0 0 10000 0 10000" rgba="0.8 0.8 0.8 1.0">
            <visual mesh="kuka/visual/link0.obj" pos="0 0 0" quat="1 0 0 0"/>
            <collision contacts="kuka/contacts/link0.txt" pos="0 0 0" quat="1 0 0 0"/>
        </body>
        <link name="kuka_link1">
            <joint name="kuka_joint1" type="revolute" axis="0 0 1" pos="0 0 14.75" quat="1 0 0 0" lim="-2.97 2.97"/>
            <body name="kuka_link1" type="abstract" pos="0 0 0" quat="1 0 0 0" mass="2700" inertia="10000 0 0 10000 0 10000" rgba="0.9 0.9 0.9 1.0">
                <visual mesh="kuka/visual/link1.obj" pos="0 0 0" quat="1 0 0 0"/>
                <collision contacts="kuka/contacts/link1.txt" pos="0 0 0" quat="1 0 0 0"/>
            </body>
            <link name="kuka_link2">
                <joint name="kuka_joint2" type="revolute" axis="0 1 0" pos="0 -1.05 19.25" quat="1 0 0 0" lim="-2.09 2.09"/>
                <body name="kuka_link2" type="abstract" pos="0 0 0" quat="1 0 0 0" mass="2730" inertia="10000 0 0 10000 0 10000" rgba="0.8 0.8 0.8 1.0">
                    <visual mesh="kuka/visual/link2.obj" pos="0 0 0" quat="1 0 0 0"/>
                    <collision contacts="kuka/contacts/link2.txt" pos="0 0 0" quat="1 0 0 0"/>
                </body>
                <link name="kuka_link3">
                    <joint name="kuka_joint3" type="revolute" axis="0 0 1" pos="0 1.05 20.75" quat="1 0 0 0" lim="-2.97 2.97"/>
                    <body name="kuka_link3" type="abstract" pos="0 0 0" quat="1 0 0 0" mass="2040" inertia="10000 0 0 10000 0 10000" rgba="0.9 0.9 0.9 1.0">
                        <visual mesh="kuka/visual/link3.obj" pos="0 0 0" quat="1 0 0 0"/>
                        <collision contacts="kuka/contacts/link3.txt" pos="0 0 0" quat="1 0 0 0"/>
                    </body>
                    <link name="kuka_link4">
                        <joint name="kuka_joint4" type="revolute" axis="0 -1 0" pos="0 1.05 19.25" quat="1 0 0 0" lim="-2.09 2.09"/>
                        <body name="kuka_link4" type="abstract" pos="0 0 0" quat="1 0 0 0" mass="2080" inertia="10000 0 0 10000 0 10000" rgba="0.8 0.8 0.8 1.0">
                            <visual mesh="kuka/visual/link4.obj" pos="0 0 0" quat="1 0 0 0"/>
                            <collision contacts="kuka/contacts/link4.txt" pos="0 0 0" quat="1 0 0 0"/>
                        </body>
                        <link name="kuka_link5">
                            <joint name="kuka_joint5" type="revolute" axis="0 0 1" pos="0 -1.05 20.75" quat="1 0 0 0" lim="-2.97 2.97"/>
                            <body name="kuka_link5" type="abstract" pos="0 0 0" quat="1 0 0 0" mass="3000" inertia="10000 0 0 10000 0 10000" rgba="0.9 0.9 0.9 1.0">
                                <visual mesh="kuka/visual/link5.obj" pos="0 0 0" quat="1 0 0 0"/>
                                <collision contacts="kuka/contacts/link5.txt" pos="0 0 0" quat="1 0 0 0"/>
                            </body>
                            <link name="kuka_link6">
                                <joint name="kuka_joint6" type="revolute" axis="0 1 0" pos="0 -7.07 19.25" quat="1 0 0 0" lim="-2.09 2.09"/>
                                <body name="kuka_link6" type="abstract" pos="0 0 0" quat="1 0 0 0" mass="1300" inertia="10000 0 0 10000 0 10000" rgba="0.8 0.8 0.8 1.0">
                                    <visual mesh="kuka/visual/link6.obj" pos="0 0 0" quat="1 0 0 0"/>
                                    <collision contacts="kuka/contacts/link6.txt" pos="0 0 0" quat="1 0 0 0"/>
                                </body>
                                <link name="kuka_link7">
                                    <joint name="kuka_joint7" type="revolute" axis="0 0 1" pos="0 7.07 9.1" quat="1 0 0 0" lim="-3.05 3.05"/>
                                    <body name="kuka_link7" type="abstract" pos="0 0 0" quat="1 0 0 0" mass="200" inertia="10000 0 0 10000 0 10000" rgba="0.9 0.9 0.9 1.0">
                                        <visual mesh="kuka/visual/link7.obj" pos="0 0 0" quat="1 0 0 0"/>
                                        <collision contacts="kuka/contacts/link7.txt" pos="0 0 0" quat="1 0 0 0"/>
                                    </body>
                                    <link name="kuka_link8">
                                        <joint name="kuka_joint8" type="fixed" pos="0 0 3.08" quat="1 0 0 0"/>
                                        <body name="kuka_link8" type="sphere" radius="0.01" pos="0 0 0" quat="1 0 0 0" rgba="0.9 0.9 0.9 1.0"/>
                                    </link>
                                </link>
                            </link>
                        </link>
                    </link>
                </link>
            </link>
        </link>
    </link>
</robot>
'''
    return string


def get_ur5e_arm_string(pos, quat):
    string = f'''
<robot>
    <link name="world_joint">
        <joint name="world_joint" type="fixed" pos="{arr_to_str(pos)}" quat="{arr_to_str(quat)}"/>
        <body name= "base_link" type = "abstract" pos = "0.0 0.0 0.0" quat = "1.0 0.0 0.0 0.0" mass = "4.0" inertia = "0.00443333156 0.00443333156 0.0072" rgba = "0.7 0.7 0.7 1.0">
            <visual mesh = "ur5e/visual/base.obj" pos = "0.0 0.0 0.0" quat = "1.0 0.0 0.0 0.0"/>
            <collision contacts = "ur5e/contacts/base.txt" pos = "0.0 0.0 0.0" quat = "1.0 0.0 0.0 0.0"/>
        </body>
        <link name="shoulder_pan_joint">
            <joint name = "shoulder_pan_joint" type="revolute" pos="0.0 0.0 8.9159" quat="1.0 0.0 0.0 0.0" axis="0.0 0.0 1.0" lim="-6.28318530718 6.28318530718" damping="0.0"/>
            <body name= "shoulder_link" type = "abstract" pos = "0.0 0.0 0.0" quat = "0.0 0.7071067811865476 0.0 -0.7071067811865476" mass = "3.7" inertia = "0.00666 0.010267495893 0.010267495893" rgba = "0.7 0.7 0.7 1.0">
                <visual mesh = "ur5e/visual/shoulder.obj" pos = "0.0 0.0 0.0" quat = "0.0 0.7071067811865475 0.0 -0.7071067811865475"/>
                <collision contacts = "ur5e/contacts/shoulder.txt" pos = "0.0 0.0 0.0" quat = "0.0 0.7071067811865475 0.0 -0.7071067811865475"/>
            </body>
            <link name="shoulder_lift_joint">
                <joint name = "shoulder_lift_joint" type="revolute" pos="0.0 13.585 0.0" quat="0.7071067811882787 0.0 0.7071067811848163 0.0" axis="0.0 1.0 0.0" lim="-6.28318530718 6.28318530718" damping="0.0"/>
                <body name= "upper_arm_link" type = "abstract" pos = "0.0 0.0 28.0" quat = "0.0 0.7071067811865476 0.0 -0.7071067811865476" mass = "8.393" inertia = "0.0151074 0.22689067591 0.22689067591" rgba = "0.7 0.7 0.7 1.0">
                    <visual mesh = "ur5e/visual/upperarm.obj" pos = "27.999999999999993 0.0 0.0" quat = "0.0 0.7071067811865475 0.0 -0.7071067811865475"/>
                    <collision contacts = "ur5e/contacts/upperarm.txt" pos = "27.999999999999993 0.0 0.0" quat = "0.0 0.7071067811865475 0.0 -0.7071067811865475"/>
                </body>
                <link name="elbow_joint">
                    <joint name = "elbow_joint" type="revolute" pos="0.0 -11.97 42.5" quat="1.0 0.0 0.0 0.0" axis="0.0 1.0 0.0" lim="-3.14159265359 3.14159265359" damping="0.0"/>
                    <body name= "forearm_link" type = "abstract" pos = "0.0 0.0 25.0" quat = "0.0 0.7071067811865476 0.0 -0.7071067811865476" mass = "2.275" inertia = "0.004095 0.049443313556 0.049443313556" rgba = "0.7 0.7 0.7 1.0">
                        <visual mesh = "ur5e/visual/forearm.obj" pos = "24.999999999999993 0.0 0.0" quat = "0.0 0.7071067811865475 0.0 -0.7071067811865475"/>
                        <collision contacts = "ur5e/contacts/forearm.txt" pos = "24.999999999999993 0.0 0.0" quat = "0.0 0.7071067811865475 0.0 -0.7071067811865475"/>
                    </body>
                    <link name="wrist_1_joint">
                        <joint name = "wrist_1_joint" type="revolute" pos="0.0 0.0 39.225" quat="0.7071067811882787 0.0 0.7071067811848163 0.0" axis="0.0 1.0 0.0" lim="-6.28318530718 6.28318530718" damping="0.0"/>
                        <body name= "wrist_1_link" type = "abstract" pos = "0.0 0.0 0.0" quat = "1.0 0.0 0.0 0.0" mass = "1.219" inertia = "0.111172755531 0.111172755531 0.21942" rgba = "0.7 0.7 0.7 1.0">
                            <visual mesh = "ur5e/visual/wrist1.obj" pos = "0.0 0.0 0.0" quat = "1.0 0.0 0.0 0.0"/>
                            <collision contacts = "ur5e/contacts/wrist1.txt" pos = "0.0 0.0 0.0" quat = "1.0 0.0 0.0 0.0"/>
                        </body>
                        <link name="wrist_2_joint">
                            <joint name = "wrist_2_joint" type="revolute" pos="0.0 12.7 0.0" quat="1.0 0.0 0.0 0.0" axis="0.0 0.0 1.0" lim="-6.28318530718 6.28318530718" damping="0.0"/>
                            <body name= "wrist_2_link" type = "abstract" pos = "0.0 0.0 0.0" quat = "1.0 0.0 0.0 0.0" mass = "1.219" inertia = "0.111172755531 0.111172755531 0.21942" rgba = "0.7 0.7 0.7 1.0">
                                <visual mesh = "ur5e/visual/wrist2.obj" pos = "0.0 0.0 0.0" quat = "1.0 0.0 0.0 0.0"/>
                                <collision contacts = "ur5e/contacts/wrist2.txt" pos = "0.0 0.0 0.0" quat = "1.0 0.0 0.0 0.0"/>
                            </body>
                            <link name="wrist_3_joint">
                                <joint name = "wrist_3_joint" type="revolute" pos="0.0 0.0 10.0" quat="1.0 0.0 0.0 0.0" axis="0.0 1.0 0.0" lim="-6.28318530718 6.28318530718" damping="0.0"/>
                                <body name= "wrist_3_link" type = "abstract" pos = "0.0 0.0 0.0" quat = "1.0 0.0 0.0 0.0" mass = "0.1879" inertia = "0.0171364731454 0.0171364731454 0.033822" rgba = "0.7 0.7 0.7 1.0">
                                    <visual mesh = "ur5e/visual/wrist3.obj" pos = "0.0 0.0 0.0" quat = "1.0 0.0 0.0 0.0"/>
                                    <collision contacts = "ur5e/contacts/wrist3.txt" pos = "0.0 0.0 0.0" quat = "1.0 0.0 0.0 0.0"/>
                                </body>
                                <link name="ee_fixed_joint">
                                    <joint name="ee_fixed_joint" type="fixed" pos="0.0 8.23 0.0" quat="0.7071067811882787 0.0 0.0 0.7071067811848163"/>
                                    <body name="ee_link" type="sphere" radius="0.01" pos="0 0 0" quat="1 0 0 0" rgba="0.9 0.9 0.9 1.0"/>
                                </link>
                            </link>
                        </link>
                    </link>
                </link>
            </link>
        </link>
    </link>
</robot>
'''
    return string


def get_arm_string(arm_type, pos, quat, suffix=None):
    if arm_type == 'xarm7':
        string = get_xarm7_arm_string(pos, quat)
    elif arm_type == 'panda':
        string = get_panda_arm_string(pos, quat)
    elif arm_type == 'kuka':
        string = get_kuka_arm_string(pos, quat)
    elif arm_type == 'ur5e':
        string = get_ur5e_arm_string(pos, quat)
    else:
        raise ValueError('Unknown arm type: {}'.format(arm_type))
    return add_suffix(string, suffix)


def get_arm_eef_joint(arm_type):
    if arm_type == 'xarm7':
        return 'joint7'
    elif arm_type == 'panda':
        return 'panda_joint8'
    elif arm_type == 'kuka':
        return 'kuka_joint8'
    elif arm_type == 'ur5e':
        return 'ee_fixed_joint'
    else:
        raise NotImplementedError


def get_arm_joints(arm_type):
    if arm_type == 'xarm7':
        return ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'joint7']
    elif arm_type == 'panda':
        return ['panda_joint1', 'panda_joint2', 'panda_joint3', 'panda_joint4', 'panda_joint5', 'panda_joint6', 'panda_joint7']
    elif arm_type == 'kuka':
        return ['kuka_joint1', 'kuka_joint2', 'kuka_joint3', 'kuka_joint4', 'kuka_joint5', 'kuka_joint6', 'kuka_joint7']
    elif arm_type == 'ur5e':
        return ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
    else:
        raise NotImplementedError
