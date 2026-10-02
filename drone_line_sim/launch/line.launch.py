"""The main line in Gazebo: world, robot, bridge and controllers (plan P5.4).

    ros2 launch drone_line_sim line.launch.py [gui:=false] [dev:=true]

  * **gz-harmonic server** on the generated world, `gz sim -s -r` (headless, running). With
    `gui:=true` a client is started after the server has had time to load.
  * **ros_gz bridge**: `/clock`, and Gazebo's own joint states (`/gz/joint_states`, from the
    JointStatePublisher plugin in the description). `/joint_states` is the hardware interface's
    view and comes from joint_state_broadcaster, so the two can be compared.
  * **robot_state_publisher and spawn** for the one robot, `line` (docs/decisions.md D-006: the
    SCARAs and the gang heads share a controller manager, so they are one model).
  * **controller spawners**, for exactly the controllers `config/controllers.yaml` defines —
    the generated file is read, so a head added to the spec is spawned with no launch change.
  * `use_sim_time` on every ROS node.

`dev:=true|false` picks the spec: `dev:=true` runs on the placeholder overlay
(worlds/main_line.dev.sdf, models/dev, config/controllers.dev.yaml) and says so on the console;
nothing from it is a deliverable (CLAUDE.md P0). It defaults to true because the base spec
still has null limits and poses, so `dev:=false` cannot generate anything yet. Every file it
needs is generated:
`tools/checks/line_smoke.sh` regenerates them all and rebuilds the package before launching.

Gazebo's environment is the one ~/ros2_ws/src/drone-assembly-line uses (read only): the model
directory on the resource path, and the ROS lib directory on both the system-plugin path and
LD_LIBRARY_PATH, which is where gz_ros2_control-system lives. That launch sets nothing else — no
GPU or Mesa variables — and a headless server renders nothing, so neither does this one.
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction,
                            RegisterEventHandler, Shutdown, TimerAction)
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

WORLD = 'main_line'                 # tools/gen_world.py WORLD_NAME
ROBOT = 'line'                      # the one robot: urdf/line.urdf.xacro
ROS_LIB = '/opt/ros/jazzy/lib'      # where gz_ros2_control-system lives
GUI_DELAY_S = 8.0                   # let the server load the world before a client connects


def _is(value: str) -> bool:
    return value.strip().lower() in ('true', '1', 'yes')


def _controller_names(controllers_file: str) -> list[str]:
    """Every controller the generated config defines, the broadcaster first."""
    with open(controllers_file) as f:
        manager = yaml.safe_load(f)['controller_manager']['ros__parameters']
    names = [k for k, v in manager.items() if isinstance(v, dict) and 'type' in v]
    return sorted(names, key=lambda n: n != 'joint_state_broadcaster')


def _launch(context):
    share = get_package_share_directory('drone_line_sim')
    dev = _is(LaunchConfiguration('dev').perform(context))
    gui = _is(LaunchConfiguration('gui').perform(context))
    suffix = '.dev' if dev else ''

    world = os.path.join(share, 'worlds', f'{WORLD}{suffix}.sdf')
    models = os.path.join(share, 'models', 'dev' if dev else '')
    controllers = os.path.join(share, 'config', f'controllers{suffix}.yaml')
    # xacro.load_yaml reads one file and knows nothing about the overlay, so a dev run reads the
    # merged spec that `tools/spec.py --dev --dump` wrote.
    spec = os.path.join(share, 'config', 'line_spec.merged.dev.yaml' if dev else 'line_spec.yaml')
    missing = [p for p in (world, controllers, spec) if not os.path.isfile(p)]
    if missing:
        raise RuntimeError(
            f'missing generated file(s): {missing}. Run tools/checks/line_smoke.sh, or the '
            'generators (gen_models, gen_world, gen_controllers, spec --dump) and colcon build.')

    gz_env = {
        'GZ_SIM_RESOURCE_PATH': ':'.join(
            p for p in (models, os.environ.get('GZ_SIM_RESOURCE_PATH')) if p),
        'GZ_SIM_SYSTEM_PLUGIN_PATH': ':'.join(
            p for p in (ROS_LIB, os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH')) if p),
        'LD_LIBRARY_PATH': ':'.join(p for p in (ROS_LIB, os.environ.get('LD_LIBRARY_PATH')) if p),
    }

    actions = []
    if dev:
        actions.append(LogInfo(msg='line.launch: DEV MODE — running on the placeholder overlay '
                                   f'({os.path.basename(spec)}); nothing here is a deliverable'))

    gz_server = ExecuteProcess(cmd=['gz', 'sim', '-s', '-r', '-v', '2', world],
                               additional_env=gz_env, output='screen')
    actions += [
        gz_server,
        # The server is the line: when it goes, everything else goes with it.
        RegisterEventHandler(OnProcessExit(target_action=gz_server, on_exit=[Shutdown()])),
    ]
    if gui:
        actions.append(TimerAction(period=GUI_DELAY_S, actions=[
            ExecuteProcess(cmd=['gz', 'sim', '-g'], additional_env=gz_env, output='screen')]))

    robot_description = ParameterValue(Command([
        FindExecutable(name='xacro'), ' ',
        os.path.join(share, 'urdf', 'line.urdf.xacro'),
        ' spec_file:=', spec, ' controllers_file:=', controllers,
    ]), value_type=str)
    sim_time = {'use_sim_time': True}

    robot_state_publisher = Node(
        package='robot_state_publisher', executable='robot_state_publisher', output='screen',
        parameters=[{'robot_description': robot_description}, sim_time])

    spawn = Node(
        package='ros_gz_sim', executable='create', output='screen',
        arguments=['-topic', 'robot_description', '-name', ROBOT, '-world', WORLD])

    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', output='screen',
        parameters=[sim_time],
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                   f'/world/{WORLD}/model/{ROBOT}/joint_state'
                   '@sensor_msgs/msg/JointState[gz.msgs.Model'],
        remappings=[(f'/world/{WORLD}/model/{ROBOT}/joint_state', '/gz/joint_states')])

    # One spawner for every controller: it loads them, then configures and activates them in
    # order, so the broadcaster (first) is running before the trajectory controllers.
    spawner = Node(
        package='controller_manager', executable='spawner', output='screen',
        arguments=[*_controller_names(controllers), '--param-file', controllers,
                   '--controller-manager-timeout', '60'],
        parameters=[sim_time])

    actions += [
        bridge,
        robot_state_publisher,
        spawn,
        RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=[spawner])),
    ]
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='false',
                              description='Start the Gazebo GUI (never during verification)'),
        DeclareLaunchArgument('dev', default_value='true',
                              description='true: placeholder overlay (dev worlds, models and '
                                          'controllers); false: the base spec alone'),
        OpaqueFunction(function=_launch),
    ])
