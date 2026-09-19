import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro

# One entry per robot station.  The namespace also becomes the Gazebo model name.
# yaw: spawn rotation about Z (radians). lerobot_1 faces +Y (into table) so
# all 4 of its bins fall in the arm's forward hemisphere.
import math as _math
ARMS = [
    {'ns': 'screw_robot',  'x': -1.2, 'y':  0.1, 'z': 0.9, 'yaw': 0.0},
    {'ns': 'lerobot_1',    'x': -1.1, 'y': -0.4, 'z': 0.9, 'yaw': _math.pi / 2},
    {'ns': 'lerobot_2',    'x': -0.6, 'y':  0.1, 'z': 0.9, 'yaw': 0.0},
    {'ns': 'solder_robot', 'x':  0.6, 'y':  0.1, 'z': 0.9, 'yaw': 0.0},
    {'ns': 'lerobot_3',    'x':  1.2, 'y':  0.1, 'z': 0.9, 'yaw': 0.0},
]

_JOINTS = ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll', 'gripper']


def _write_arm_yaml(ns: str) -> str:
    """Write a namespace-scoped controller YAML to /tmp and return its path.

    rclcpp matches YAML top-level keys against the node's *fully-qualified* name,
    so a bare 'controller_manager:' key does not match '/screw_robot/controller_manager'.
    Using '/<ns>/...' as the key makes the match unambiguous.
    """
    data = {
        f'/{ns}/controller_manager': {
            'ros__parameters': {
                'update_rate': 100,
                'joint_state_broadcaster': {
                    'type': 'joint_state_broadcaster/JointStateBroadcaster',
                },
                'so101_arm_controller': {
                    'type': 'joint_trajectory_controller/JointTrajectoryController',
                },
            }
        },
        f'/{ns}/joint_state_broadcaster': {
            'ros__parameters': {
                'use_local_topics': True,
            }
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
    pkg_share   = get_package_share_directory('drone_assembly_cell')
    so101_pkg   = get_package_share_directory('so101_description')
    moveit_pkg  = get_package_share_directory('assembly_line_moveit_config')
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

    rviz_config       = os.path.join(pkg_share, 'rviz', 'cell.rviz')
    xacro_file        = os.path.join(so101_pkg, 'urdf', 'so101.urdf.xacro')
    xacro_screwdriver = os.path.join(so101_pkg, 'urdf', 'so101_screwdriver.urdf.xacro')

    # ── Physics tuning — read once, applied to every arm ──────────────────
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

    # ── Launch arguments ───────────────────────────────────────────────────
    gui_arg  = DeclareLaunchArgument('gui',  default_value='true',
                                     description='Start the Gazebo GUI')
    rviz_arg = DeclareLaunchArgument('rviz', default_value='false',
                                     description='Launch RViz2')
    gui  = LaunchConfiguration('gui')
    rviz = LaunchConfiguration('rviz')

    # ── Gazebo server + GUI ────────────────────────────────────────────────
    # Start paused (-p) so no physics steps run while robot models are being
    # spawned.  Adding DART bodies during a live physics step causes ODE to
    # receive zero-initialised quaternions → "dxNormalize4" assertion crash.
    # Physics is unpaused explicitly below, after all five arms are in the
    # world and their controller_managers are ready.
    gz_server = ExecuteProcess(
        cmd=['gz', 'sim', '-s', '-r', world_file],
        additional_env=gz_env, output='screen',
    )
    gz_gui = TimerAction(period=8.0, actions=[
        ExecuteProcess(cmd=['gz', 'sim', '-g'], additional_env=gz_env,
                       output='screen', condition=IfCondition(gui))
    ])

    # ── ROS-GZ bridge ──────────────────────────────────────────────────────
    contact_bridges = [
        f'/{arm["ns"]}/gripper/contact@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts'
        for arm in ARMS
    ]
    gz_bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        name='ros_gz_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/world/frame_assembly_cell/pose/info'
            '@geometry_msgs/msg/PoseArray[gz.msgs.Pose_V',
            '/world/frame_assembly_cell/dynamic_pose/info'
            '@geometry_msgs/msg/PoseArray[gz.msgs.Pose_V',
            *contact_bridges,
        ],
        output='screen',
    )

    # ── Screw spawner service ──────────────────────────────────────────────
    screw_spawner = Node(
        package='drone_assembly_cell', executable='screw_spawner.py',
        name='screw_spawner', output='screen',
    )

    # ── Virtual conveyor transport service ────────────────────────────────
    conveyor_node = Node(
        package='drone_assembly_cell', executable='conveyor_node.py',
        name='conveyor_node', output='screen',
    )

    actions = [gui_arg, rviz_arg, gz_server, gz_bridge, gz_gui,
               screw_spawner, conveyor_node]

    # ── MoveIt2 shared config (loaded once, used per-arm below) ───────────
    kinematics_params   = _load_yaml(os.path.join(moveit_pkg, 'config', 'kinematics.yaml'))
    joint_limits_params = _load_yaml(os.path.join(moveit_pkg, 'config', 'joint_limits.yaml'))
    moveit_params       = _load_yaml(os.path.join(moveit_pkg, 'config', 'moveit.yaml'))

    # ── Per-arm: RSP + Gazebo spawn + controllers ──────────────────────────
    # Model spawns are staggered 3 s apart so each arm's gz_ros2_control
    # plugin can register before the next model is injected.
    # Controller spawners are staggered 6 s apart (one arm at a time) to
    # avoid contention on the controller_manager lock; JSB fires 2 s before
    # the arm trajectory controller within each arm's window.
    #   screw_robot  jsb=13s arm_ctrl=16s
    #   lerobot_1    jsb=25s arm_ctrl=28s
    #   lerobot_2    jsb=37s arm_ctrl=40s
    #   solder_robot jsb=49s arm_ctrl=52s
    #   lerobot_3    jsb=61s arm_ctrl=64s
    for i, arm in enumerate(ARMS):
        ns         = arm['ns']
        spawn_yaw  = arm.get('yaw', 0.0)
        spawn_t    = 5.0  + i * 3.0   # 5, 8, 11, 14, 17 s
        jsb_t      = 13.0 + i * 12.0  # 13, 25, 37, 49, 61 s
        arm_ctrl_t = 16.0 + i * 12.0  # 16, 28, 40, 52, 64 s

        yaml_path  = _write_arm_yaml(ns)

        # screw_robot uses the screwdriver-variant xacro (adds tip link/joint)
        arm_xacro = xacro_screwdriver if ns == 'screw_robot' else xacro_file
        xacro_mappings = {'namespace': ns, 'controllers_yaml': yaml_path, **physics_mappings}
        if ns == 'screw_robot':
            xacro_mappings['screwdriver'] = 'true'
        robot_desc = xacro.process_file(arm_xacro, mappings=xacro_mappings).toxml()

        # robot_state_publisher — unique namespace + frame_prefix avoids TF clashes
        rsp = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace=ns,
            parameters=[{
                'robot_description': robot_desc,
                'frame_prefix': ns + '/',
            }],
            output='screen',
        )

        spawn = TimerAction(period=spawn_t, actions=[
            Node(
                package='ros_gz_sim', executable='create',
                arguments=[
                    '-name', ns,
                    '-string', robot_desc,
                    '-x', str(arm['x']),
                    '-y', str(arm['y']),
                    '-z', str(arm['z']),
                    '-R', '0.0', '-P', '0.0', '-Y', str(spawn_yaw),
                ],
                output='screen',
            )
        ])

        load_jsb = TimerAction(period=jsb_t, actions=[
            Node(
                package='controller_manager', executable='spawner',
                arguments=[
                    'joint_state_broadcaster',
                    '--controller-manager', f'/{ns}/controller_manager',
                ],
                output='screen',
            )
        ])

        load_arm_ctrl = TimerAction(period=arm_ctrl_t, actions=[
            Node(
                package='controller_manager', executable='spawner',
                arguments=[
                    'so101_arm_controller',
                    '--controller-manager', f'/{ns}/controller_manager',
                    '--param-file', yaml_path,
                ],
                output='screen',
            )
        ])

        actions += [rsp, spawn, load_jsb, load_arm_ctrl]

        # move_group at T=70s — after all controllers up; no RSP (already running)
        srdf_path = os.path.join(moveit_pkg, 'config', f'{ns}.srdf')
        with open(srdf_path, 'r') as fh:
            srdf_content = fh.read()
        load_move_group = TimerAction(period=70.0, actions=[
            Node(
                package='moveit_ros_move_group',
                executable='move_group',
                namespace=ns,
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
        actions.append(load_move_group)

        if ns != 'screw_robot':
            load_attach = TimerAction(period=arm_ctrl_t + 2.0, actions=[
                Node(
                    package='drone_assembly_cell',
                    executable='gripper_attach_node.py',
                    name=f'gripper_attach_{ns}',
                    parameters=[{
                        'namespace': ns,
                        'world': 'frame_assembly_cell',
                        'base_x': arm['x'],
                        'base_y': arm['y'],
                        'base_z': arm['z'],
                        'spawn_yaw': spawn_yaw,
                    }],
                    output='screen',
                )
            ])
            actions.append(load_attach)

            load_pps = TimerAction(period=arm_ctrl_t + 6.0, actions=[
                Node(
                    package='drone_assembly_cell',
                    executable='pick_place_server.py',
                    name=f'pick_place_{ns}',
                    parameters=[{
                        'namespace': ns,
                        'base_x': arm['x'],
                        'base_y': arm['y'],
                        'base_z': arm['z'],
                    }],
                    output='screen',
                )
            ])
            actions.append(load_pps)

            # Orchestrator for lerobot_1 only (bin A pick demo)
            if ns == 'lerobot_1':
                load_orch = TimerAction(period=arm_ctrl_t + 10.0, actions=[
                    Node(
                        package='drone_assembly_cell',
                        executable='assembly_orchestrator.py',
                        name='assembly_orchestrator',
                        parameters=[{'namespace': ns}],
                        output='screen',
                    )
                ])
                actions.append(load_orch)

    # ── RViz2 (optional, after all controllers are up at T≈64s) ───────────
    rviz_node = TimerAction(period=70.0, actions=[
        Node(
            package='rviz2', executable='rviz2', name='rviz2',
            arguments=['-d', rviz_config], output='screen',
            condition=IfCondition(rviz),
        )
    ])
    actions.append(rviz_node)

    return LaunchDescription(actions)
