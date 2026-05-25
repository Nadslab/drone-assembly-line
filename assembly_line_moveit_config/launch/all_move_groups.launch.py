"""Launch MoveIt 2 move_group for all 5 assembly arms simultaneously.

Each arm runs in its own ROS namespace so topics/services do not collide:
  /screw_robot/move_action     /screw_robot/compute_ik
  /lerobot_1/move_action       /lerobot_1/compute_ik
  /lerobot_2/move_action       /lerobot_2/compute_ik
  /solder_robot/move_action    /solder_robot/compute_ik
  /lerobot_3/move_action       /lerobot_3/compute_ik

Workspace separation analysis (nominal operation, all joints within limits):
  Arms spaced 0.60 m apart in X; max arm reach ≈ 0.40 m.
  Each arm's bins lie within ±0.20 m of its station in X, well inside
  the ±0.30 m midpoint boundary to each neighbour.  No two arms can
  simultaneously reach each other's nominal bin positions.
"""
import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
import xacro

ARMS = ['screw_robot', 'lerobot_1', 'lerobot_2', 'solder_robot', 'lerobot_3']


def _load_yaml(path: str) -> dict:
    with open(path, 'r') as fh:
        data = yaml.safe_load(fh)
    if '/**' in data:
        return data['/**']['ros__parameters']
    return data


def generate_launch_description():
    pkg_share = get_package_share_directory('assembly_line_moveit_config')
    so101_pkg  = get_package_share_directory('so101_description')
    xacro_file = os.path.join(so101_pkg, 'urdf', 'so101.urdf.xacro')

    # Shared config (loaded once, passed to all arms)
    kinematics_params   = _load_yaml(os.path.join(pkg_share, 'config', 'kinematics.yaml'))
    joint_limits_params = _load_yaml(os.path.join(pkg_share, 'config', 'joint_limits.yaml'))
    moveit_params       = _load_yaml(os.path.join(pkg_share, 'config', 'moveit.yaml'))

    use_sim_time = LaunchConfiguration('use_sim_time')

    actions = [
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use /clock from simulation',
        ),
    ]

    for arm in ARMS:
        robot_desc = xacro.process_file(
            xacro_file, mappings={'namespace': arm}
        ).toxml()
        robot_description = {'robot_description': robot_desc}

        srdf_path = os.path.join(pkg_share, 'config', f'{arm}.srdf')
        with open(srdf_path, 'r') as fh:
            srdf_content = fh.read()
        robot_description_semantic = {'robot_description_semantic': srdf_content}

        rsp = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace=arm,
            name='robot_state_publisher',
            output='screen',
            parameters=[robot_description, {'use_sim_time': use_sim_time}],
        )

        mg = Node(
            package='moveit_ros_move_group',
            executable='move_group',
            namespace=arm,
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

        actions += [rsp, mg]

    return LaunchDescription(actions)
