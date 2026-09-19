"""P1 spike: one-axis cart on a rail under ros2_control in Gazebo.

Mirrors gz_ros2_control_demos/launch/cart_example_position.launch.py.
Headless by default (gui:=false runs the Gazebo server only).
"""

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction,
                            RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _gazebo(context):
    gui = LaunchConfiguration('gui').perform(context).lower() in ('true', '1')
    gz_args = '-r -v 1 empty.sdf' if gui else '-r -s -v 1 empty.sdf'
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])),
        launch_arguments=[('gz_args', gz_args), ('on_exit_shutdown', 'true')],
    )]


def generate_launch_description():
    controllers_file = PathJoinSubstitution(
        [FindPackageShare('drone_line_sim'), 'config', 'spike_controllers.yaml'])

    robot_description = ParameterValue(Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]), ' ',
        PathJoinSubstitution([FindPackageShare('drone_line_sim'), 'urdf', 'spike_axis.urdf.xacro']),
        ' controllers_file:=', controllers_file,
    ]), value_type=str)

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
    )

    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', 'robot_description', '-name', 'spike_axis', '-allow_renaming', 'true'],
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
    )

    joint_trajectory_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_trajectory_controller', '--param-file', controllers_file],
    )

    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='false',
                              description='Launch the Gazebo GUI (never during verification)'),
        OpaqueFunction(function=_gazebo),
        clock_bridge,
        robot_state_publisher,
        gz_spawn_entity,
        RegisterEventHandler(OnProcessExit(
            target_action=gz_spawn_entity,
            on_exit=[joint_state_broadcaster_spawner])),
        RegisterEventHandler(OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[joint_trajectory_controller_spawner])),
    ])
