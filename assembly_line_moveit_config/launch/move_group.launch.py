"""Parametrized MoveIt 2 move_group launcher for any of the 5 assembly arms.

Usage:
  ros2 launch assembly_line_moveit_config move_group.launch.py arm_name:=lerobot_1
  ros2 launch assembly_line_moveit_config move_group.launch.py arm_name:=screw_robot
"""
import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro

VALID_ARMS = {
    'screw_robot', 'lerobot_1', 'lerobot_2', 'solder_robot', 'lerobot_3'
}


def _load_yaml(path: str) -> dict:
    """Load YAML, unwrapping /**:ros__parameters: if present."""
    with open(path, 'r') as fh:
        data = yaml.safe_load(fh)
    if '/**' in data:
        return data['/**']['ros__parameters']
    return data


def launch_setup(context, *args, **kwargs):
    arm_name     = LaunchConfiguration('arm_name').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context)

    if arm_name not in VALID_ARMS:
        raise ValueError(
            f"arm_name '{arm_name}' not in {VALID_ARMS}"
        )

    pkg_share = get_package_share_directory('assembly_line_moveit_config')
    so101_pkg  = get_package_share_directory('so101_description')

    # ── Robot description ─────────────────────────────────────────────
    xacro_file   = os.path.join(so101_pkg, 'urdf', 'so101.urdf.xacro')
    robot_desc   = xacro.process_file(
        xacro_file, mappings={'namespace': arm_name}
    ).toxml()
    robot_description = {'robot_description': robot_desc}

    # ── Semantic description (per-arm SRDF) ───────────────────────────
    srdf_path = os.path.join(pkg_share, 'config', f'{arm_name}.srdf')
    with open(srdf_path, 'r') as fh:
        srdf_content = fh.read()
    robot_description_semantic = {'robot_description_semantic': srdf_content}

    # ── Shared config ─────────────────────────────────────────────────
    kinematics_params   = _load_yaml(os.path.join(pkg_share, 'config', 'kinematics.yaml'))
    joint_limits_params = _load_yaml(os.path.join(pkg_share, 'config', 'joint_limits.yaml'))
    moveit_params       = _load_yaml(os.path.join(pkg_share, 'config', 'moveit.yaml'))

    sim_time_param = {'use_sim_time': use_sim_time == 'true'}

    # ── robot_state_publisher ─────────────────────────────────────────
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=arm_name,
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description, sim_time_param],
    )

    # ── move_group (namespaced so 5 instances can coexist) ────────────
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        namespace=arm_name,
        name='move_group',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_params,
            joint_limits_params,
            moveit_params,
            sim_time_param,
        ],
    )

    return [rsp_node, move_group_node]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'arm_name',
            default_value='lerobot_1',
            description='Arm namespace: screw_robot|lerobot_1|lerobot_2|solder_robot|lerobot_3',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use /clock from simulation',
        ),
        OpaqueFunction(function=launch_setup),
    ])
