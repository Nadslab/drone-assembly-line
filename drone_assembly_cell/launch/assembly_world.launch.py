import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('drone_assembly_cell')
    worlds_dir = os.path.join(pkg_share, 'worlds')
    models_dir = os.path.join(pkg_share, 'models')

    sdf_files = [f for f in os.listdir(worlds_dir) if f.endswith('.sdf')]
    if not sdf_files:
        raise RuntimeError(f'No .sdf file found in {worlds_dir}')
    world_file = os.path.join(worlds_dir, sorted(sdf_files)[0])

    gz_env = {'GZ_SIM_RESOURCE_PATH': models_dir}

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
            )
        ],
    )

    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/world/frame_assembly_cell/pose/info@geometry_msgs/msg/PoseArray[gz.msgs.Pose_V',
        ],
        output='screen',
    )

    return LaunchDescription([gz_server, gz_bridge, gz_gui])
