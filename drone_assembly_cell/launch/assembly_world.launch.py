import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    pkg_share = get_package_share_directory('drone_assembly_cell')
    so101_pkg = get_package_share_directory('so101_description')
    # Parent of so101_description share — needed so Gazebo resolves model://so101_description/...
    so101_share_parent = os.path.dirname(so101_pkg)

    worlds_dir = os.path.join(pkg_share, 'worlds')
    models_dir = os.path.join(pkg_share, 'models')

    sdf_files = [f for f in os.listdir(worlds_dir) if f.endswith('.sdf')]
    if not sdf_files:
        raise RuntimeError(f'No .sdf file found in {worlds_dir}')
    world_file = os.path.join(worlds_dir, sorted(sdf_files)[0])

    ros_lib = '/opt/ros/jazzy/lib'

    gz_env = {
        # Drone-assembly SDF models + so101 meshes (resolves model://so101_description/meshes/...)
        'GZ_SIM_RESOURCE_PATH': f'{models_dir}:{so101_share_parent}',
        # gz_ros2_control plugin lives in the ROS lib directory
        'GZ_SIM_SYSTEM_PLUGIN_PATH': ros_lib,
        # ROS lib must be on LD_LIBRARY_PATH so Gazebo can resolve the plugin's ROS dependencies
        'LD_LIBRARY_PATH': f'{ros_lib}:{os.environ.get("LD_LIBRARY_PATH", "")}',
    }

    rviz_config = os.path.join(pkg_share, 'rviz', 'cell.rviz')

    # ── Launch arguments ───────────────────────────────────────────────────
    gui_arg = DeclareLaunchArgument(
        'gui', default_value='true',
        description='Start the Gazebo GUI (set false for headless/CI runs)',
    )
    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='false',
        description='Launch RViz2 with the cell.rviz config',
    )

    gui = LaunchConfiguration('gui')
    rviz = LaunchConfiguration('rviz')

    # ── Gazebo server ──────────────────────────────────────────────────────
    gz_server = ExecuteProcess(
        cmd=['gz', 'sim', '-s', '-r', world_file],
        additional_env=gz_env,
        output='screen',
    )

    gz_gui = TimerAction(
        period=8.0,
        actions=[
            ExecuteProcess(
                cmd=['gz', 'sim', '-g'],
                additional_env=gz_env,
                output='screen',
                condition=IfCondition(gui),
            )
        ],
    )

    # ── ROS-GZ bridge ──────────────────────────────────────────────────────
    # Bridge notation:
    #   topic@ros_type[gz_type   →  subscribe from Gazebo, publish to ROS
    #   topic@ros_type]gz_type   →  subscribe from ROS, publish to Gazebo
    #   topic@ros_type@gz_type   →  bidirectional
    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        arguments=[
            # Simulation clock
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            # All-model poses (static + dynamic together, 10 Hz from SceneBroadcaster)
            '/world/frame_assembly_cell/pose/info'
            '@geometry_msgs/msg/PoseArray[gz.msgs.Pose_V',
            # High-rate dynamic poses only (physics-stepped, every sim step)
            '/world/frame_assembly_cell/dynamic_pose/info'
            '@geometry_msgs/msg/PoseArray[gz.msgs.Pose_V',
            # Gripper contact sensor — placeholder until gripper link + sensor are added.
            # The bridge sits idle if no Gazebo publisher exists on this topic yet.
            '/gripper/contact'
            '@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts',
        ],
        output='screen',
    )

    # ── SO-101 arm: robot_state_publisher ──────────────────────────────────
    xacro_file = os.path.join(so101_pkg, 'urdf', 'so101.urdf.xacro')
    robot_desc = xacro.process_file(xacro_file).toxml()

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_desc}],
        output='screen',
    )

    # ── SO-101 arm: spawn in Gazebo at (-0.3, 0, 0.9) ─────────────────────
    spawn_arm = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                arguments=[
                    '-name', 'so101',
                    '-string', robot_desc,
                    '-x', '-0.6',   # station 2 (LeRobot #1)
                    '-y', '-0.40',  # front edge of table
                    '-z', '0.9',    # table surface
                    '-R', '0.0',
                    '-P', '0.0',
                    '-Y', '0.0',
                ],
                output='screen',
            )
        ],
    )

    # ── ros2_control: load controllers after arm+plugin are ready ──────────
    load_jsb = TimerAction(
        period=12.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=[
                    'joint_state_broadcaster',
                    '--controller-manager', '/controller_manager',
                ],
                output='screen',
            )
        ],
    )

    load_arm_ctrl = TimerAction(
        period=15.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=[
                    'so101_arm_controller',
                    '--controller-manager', '/controller_manager',
                ],
                output='screen',
            )
        ],
    )

    # ── Screw spawner service ──────────────────────────────────────────────
    screw_spawner = Node(
        package='drone_assembly_cell',
        executable='screw_spawner.py',
        name='screw_spawner',
        output='screen',
    )

    # ── RViz2 (optional) ──────────────────────────────────────────────────
    rviz_node = TimerAction(
        period=18.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', rviz_config],
                output='screen',
                condition=IfCondition(rviz),
            )
        ],
    )

    return LaunchDescription([
        gui_arg,
        rviz_arg,
        gz_server,
        gz_bridge,
        gz_gui,
        rsp,
        spawn_arm,
        load_jsb,
        load_arm_ctrl,
        screw_spawner,
        rviz_node,
    ])
