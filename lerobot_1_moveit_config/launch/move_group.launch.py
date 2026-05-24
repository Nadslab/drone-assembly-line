import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def _load_yaml(path: str) -> dict:
    """Load a YAML file, unwrapping /**:ros__parameters: if present."""
    with open(path, 'r') as fh:
        data = yaml.safe_load(fh)
    # Unwrap ROS 2 params-file wrapper so the dict can be passed inline
    if '/**' in data:
        return data['/**']['ros__parameters']
    return data


def generate_launch_description():
    pkg_share = get_package_share_directory('lerobot_1_moveit_config')
    so101_pkg  = get_package_share_directory('so101_description')

    use_sim_time = LaunchConfiguration('use_sim_time')

    # ── Robot description (URDF processed for lerobot_1) ─────────────────
    xacro_file = os.path.join(so101_pkg, 'urdf', 'so101.urdf.xacro')
    robot_desc_xml = xacro.process_file(
        xacro_file,
        mappings={'namespace': 'lerobot_1'},
    ).toxml()
    robot_description = {'robot_description': robot_desc_xml}

    # ── Semantic description (SRDF) ───────────────────────────────────────
    srdf_path = os.path.join(pkg_share, 'config', 'so101.srdf')
    with open(srdf_path, 'r') as fh:
        srdf_content = fh.read()
    robot_description_semantic = {'robot_description_semantic': srdf_content}

    # ── Config files loaded as dicts ──────────────────────────────────────
    kinematics_params   = _load_yaml(os.path.join(pkg_share, 'config', 'kinematics.yaml'))
    joint_limits_params = _load_yaml(os.path.join(pkg_share, 'config', 'joint_limits.yaml'))
    moveit_params       = _load_yaml(os.path.join(pkg_share, 'config', 'moveit.yaml'))

    # ── robot_state_publisher (needed for TF / move_group) ───────────────
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            robot_description,
            {'use_sim_time': use_sim_time},
        ],
    )

    # ── move_group ────────────────────────────────────────────────────────
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        name='move_group',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_params,
            joint_limits_params,
            moveit_params,
            {'use_sim_time': use_sim_time},
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use /clock from simulation',
        ),
        rsp_node,
        move_group_node,
    ])
