"""
Development launch: LeRobot 1 only.

Spawns a single arm in the assembly world for fast iteration.
Startup is ~20s instead of 70s (no staggered timing for 5 arms).
"""

import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro

import math as _math
_NS  = 'lerobot_1'
_ARM = {'x': -1.1, 'y': -0.4, 'z': 0.9, 'yaw': _math.pi / 2}

_JOINTS = ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll', 'gripper']


def _write_arm_yaml(ns: str) -> str:
    data = {
        f'/{ns}/controller_manager': {
            'ros__parameters': {
                'update_rate': 100,
                'joint_state_broadcaster': {'type': 'joint_state_broadcaster/JointStateBroadcaster'},
                'so101_arm_controller':    {'type': 'joint_trajectory_controller/JointTrajectoryController'},
            }
        },
        f'/{ns}/joint_state_broadcaster': {
            'ros__parameters': {'use_local_topics': True}
        },
        f'/{ns}/so101_arm_controller': {
            'ros__parameters': {
                'joints': _JOINTS,
                'command_interfaces': ['position'],
                'state_interfaces': ['position', 'velocity'],
                'state_publish_rate': 50.0,
                'action_monitor_rate': 20.0,
                'allow_partial_joints_goal': False,
                'open_loop_control': True,
                'allow_integration_in_goal_trajectories': False,
            }
        },
    }
    path = f'/tmp/ros2_controllers_{ns}.yaml'
    with open(path, 'w') as fh:
        yaml.dump(data, fh, default_flow_style=False)
    return path


def _load_yaml(path: str) -> dict:
    with open(path, 'r') as fh:
        data = yaml.safe_load(fh)
    if '/**' in data:
        return data['/**']['ros__parameters']
    return data


def generate_launch_description():
    pkg_share  = get_package_share_directory('drone_assembly_cell')
    so101_pkg  = get_package_share_directory('so101_description')
    moveit_pkg = get_package_share_directory('assembly_line_moveit_config')
    so101_share_parent = os.path.dirname(so101_pkg)

    worlds_dir = os.path.join(pkg_share, 'worlds')
    models_dir = os.path.join(pkg_share, 'models')

    sdf_files = [f for f in os.listdir(worlds_dir) if f.endswith('.sdf')]
    if not sdf_files:
        raise RuntimeError(f'No .sdf file found in {worlds_dir}')
    world_file = os.path.join(worlds_dir, sorted(sdf_files)[0])

    ros_lib = '/opt/ros/jazzy/lib'
    gz_env = {
        'GZ_SIM_RESOURCE_PATH':      f'{models_dir}:{so101_share_parent}',
        'GZ_SIM_SYSTEM_PLUGIN_PATH': ros_lib,
        'LD_LIBRARY_PATH':           f'{ros_lib}:{os.environ.get("LD_LIBRARY_PATH", "")}',
    }

    xacro_file = os.path.join(so101_pkg, 'urdf', 'so101.urdf.xacro')
    yaml_path  = _write_arm_yaml(_NS)

    # ── Physics tuning ────────────────────────────────────────────────────
    _tuning_path = os.path.join(so101_pkg, 'config', 'physics_tuning.yaml')
    with open(_tuning_path) as _f:
        _tuning = yaml.safe_load(_f)
    physics_mappings = {
        'inertia_scale':      str(_tuning.get('inertia_scale',      1.0)),
        'mass_scale':         str(_tuning.get('mass_scale',         1.0)),
        'joint_damping':      str(_tuning.get('joint_damping',      0.1)),
        'joint_friction':     str(_tuning.get('joint_friction',     0.05)),
        'position_hold_gain': str(_tuning.get('position_hold_gain', 20.0)),
    }

    robot_desc = xacro.process_file(
        xacro_file, mappings={'namespace': _NS, 'controllers_yaml': yaml_path, **physics_mappings}
    ).toxml()

    srdf_path = os.path.join(moveit_pkg, 'config', f'{_NS}.srdf')
    with open(srdf_path, 'r') as fh:
        srdf_content = fh.read()

    kinematics_params   = _load_yaml(os.path.join(moveit_pkg, 'config', 'kinematics.yaml'))
    joint_limits_params = _load_yaml(os.path.join(moveit_pkg, 'config', 'joint_limits.yaml'))
    moveit_params       = _load_yaml(os.path.join(moveit_pkg, 'config', 'moveit.yaml'))

    # ── Launch arguments ───────────────────────────────────────────────────
    gui_arg  = DeclareLaunchArgument('gui',  default_value='true',
                                     description='Start the Gazebo GUI')
    rviz_arg = DeclareLaunchArgument('rviz', default_value='false',
                                     description='Launch RViz2')
    rviz_config = os.path.join(pkg_share, 'rviz', 'cell.rviz')
    gui  = LaunchConfiguration('gui')
    rviz = LaunchConfiguration('rviz')

    # ── Gazebo ─────────────────────────────────────────────────────────────
    gz_server = ExecuteProcess(
        cmd=['gz', 'sim', '-s', '-r', world_file],
        additional_env=gz_env, output='screen',
    )
    gz_gui = TimerAction(period=5.0, actions=[
        ExecuteProcess(cmd=['gz', 'sim', '-g'], additional_env=gz_env,
                       output='screen', condition=IfCondition(gui))
    ])

    # ── ROS-GZ bridge (clock + single arm contact) ─────────────────────────
    gz_bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        name='ros_gz_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            f'/{_NS}/gripper/contact@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts',
        ],
        output='screen',
    )

    # ── Robot state publisher ──────────────────────────────────────────────
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=_NS,
        parameters=[{
            'robot_description': robot_desc,
            'frame_prefix': _NS + '/',
        }],
        output='screen',
    )

    # ── Gazebo spawn ───────────────────────────────────────────────────────
    spawn = TimerAction(period=5.0, actions=[
        Node(
            package='ros_gz_sim', executable='create',
            arguments=[
                '-name', _NS,
                '-string', robot_desc,
                '-x', str(_ARM['x']),
                '-y', str(_ARM['y']),
                '-z', str(_ARM['z']),
                '-R', '0.0', '-P', '0.0', '-Y', str(_ARM['yaw']),
            ],
            output='screen',
        )
    ])

    # ── Controllers ────────────────────────────────────────────────────────
    # Single arm — no staggering needed, conservative delays only for Gazebo readiness.
    load_jsb = TimerAction(period=10.0, actions=[
        Node(
            package='controller_manager', executable='spawner',
            arguments=['joint_state_broadcaster',
                       '--controller-manager', f'/{_NS}/controller_manager'],
            output='screen',
        )
    ])
    load_arm_ctrl = TimerAction(period=13.0, actions=[
        Node(
            package='controller_manager', executable='spawner',
            arguments=['so101_arm_controller',
                       '--controller-manager', f'/{_NS}/controller_manager',
                       '--param-file', yaml_path],
            output='screen',
        )
    ])

    # ── Application nodes ──────────────────────────────────────────────────
    load_attach = TimerAction(period=15.0, actions=[
        Node(
            package='drone_assembly_cell',
            executable='gripper_attach_node.py',
            name=f'gripper_attach_{_NS}',
            parameters=[{
                'namespace': _NS,
                'world': 'frame_assembly_cell',
                'base_x': _ARM['x'],
                'base_y': _ARM['y'],
                'base_z': _ARM['z'],
                'spawn_yaw': _ARM['yaw'],
            }],
            output='screen',
        )
    ])
    load_pps = TimerAction(period=19.0, actions=[
        Node(
            package='drone_assembly_cell',
            executable='pick_place_server.py',
            name=f'pick_place_{_NS}',
            parameters=[{
                'namespace': _NS,
                'base_x': _ARM['x'],
                'base_y': _ARM['y'],
                'base_z': _ARM['z'],
            }],
            output='screen',
        )
    ])
    load_orch = TimerAction(period=21.0, actions=[
        Node(
            package='drone_assembly_cell',
            executable='assembly_orchestrator.py',
            name='assembly_orchestrator',
            parameters=[{'namespace': _NS}],
            output='screen',
        )
    ])

    load_move_group = TimerAction(period=17.0, actions=[
        Node(
            package='moveit_ros_move_group',
            executable='move_group',
            namespace=_NS,
            name='move_group',
            output='screen',
            parameters=[
                {'robot_description': robot_desc},
                {'robot_description_semantic': srdf_content},
                kinematics_params,
                joint_limits_params,
                moveit_params,
                {'use_sim_time': True},
            ],
        )
    ])

    # ── Part spawner ───────────────────────────────────────────────────────
    screw_spawner = Node(
        package='drone_assembly_cell', executable='screw_spawner.py',
        name='screw_spawner', output='screen',
    )

    rviz_node = TimerAction(period=20.0, actions=[
        Node(
            package='rviz2', executable='rviz2', name='rviz2',
            arguments=['-d', rviz_config], output='screen',
            condition=IfCondition(rviz),
        )
    ])

    return LaunchDescription([
        gui_arg, rviz_arg,
        gz_server, gz_gui, gz_bridge,
        rsp, spawn,
        load_jsb, load_arm_ctrl,
        load_attach, load_move_group, load_pps,
        load_orch,
        screw_spawner, rviz_node,
    ])
