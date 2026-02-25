import os
import tempfile

from pybullet_tools.bullet_utils import load_robot_urdf
from pybullet_tools.utils import PI, get_model_path

from robot_builder.robot_utils import BASE_TORSO_GROUP

FETCH_URDF = 'models/fetch_description/robots/fetch.urdf'
FETCH_TOOL_LINK = 'wrist_roll_link'
FETCH_GRIPPER_ROOT = 'wrist_roll_link'

FETCH_JOINT_GROUPS = {
    BASE_TORSO_GROUP: ['x', 'y', 'torso_lift_joint', 'theta'],
    'arm': ['shoulder_pan_joint', 'shoulder_lift_joint', 'upperarm_roll_joint',
            'elbow_flex_joint', 'forearm_roll_joint', 'wrist_flex_joint',
            'wrist_roll_joint'],
    'gripper': ['r_gripper_finger_joint', 'l_gripper_finger_joint'],
    'head': ['head_pan_joint', 'head_tilt_joint'],
}

FETCH_CARRY_ARM_CONF = (0, -1.0, 0, 1.5, 0, 1.3, 0)


def _virtual_base_fetch_urdf_path():
        source_path = get_model_path(FETCH_URDF)
        with open(source_path, 'r') as source_file:
                source_urdf = source_file.read()

        has_virtual_base = all(token in source_urdf for token in ['name="x"', 'name="y"', 'name="theta"'])
        if has_virtual_base:
                return source_path, None

        insertion = """
    <link name="world" />
    <link name="base_x_link" />
    <joint name="x" type="prismatic">
        <parent link="world" />
        <child link="base_x_link" />
        <axis xyz="1 0 0" />
        <limit lower="-20" upper="20" effort="1000" velocity="2.0" />
    </joint>
    <link name="base_y_link" />
    <joint name="y" type="prismatic">
        <parent link="base_x_link" />
        <child link="base_y_link" />
        <axis xyz="0 1 0" />
        <limit lower="-20" upper="20" effort="1000" velocity="2.0" />
    </joint>
    <link name="base_theta_link" />
    <joint name="theta" type="continuous">
        <parent link="base_y_link" />
        <child link="base_theta_link" />
        <axis xyz="0 0 1" />
        <limit effort="1000" velocity="2.0" />
    </joint>
    <joint name="base_to_body" type="fixed">
        <parent link="base_theta_link" />
        <child link="base_link" />
        <origin xyz="0 0 0" rpy="0 0 0" />
    </joint>
"""

        robot_open = source_urdf.find('>')
        if robot_open == -1:
                raise ValueError('Invalid Fetch URDF: missing robot tag')
        wrapped_urdf = source_urdf[:robot_open + 1] + insertion + source_urdf[robot_open + 1:]

        temp_file = tempfile.NamedTemporaryFile(
                mode='w',
                suffix='.urdf',
                prefix='fetch_virtual_base_',
                dir=os.path.dirname(source_path),
                delete=False,
        )
        with temp_file:
                temp_file.write(wrapped_urdf)
        return temp_file.name, temp_file.name


def load_fetch():
        urdf_path, temp_path = _virtual_base_fetch_urdf_path()
        try:
                return load_robot_urdf(urdf_path)
        finally:
                if temp_path is not None and os.path.exists(temp_path):
                        os.remove(temp_path)