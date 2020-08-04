import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription

from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = Path(get_package_share_directory('rj_robocup'))
    launch_dir = bringup_dir / 'launch'

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED', '1')

    grsim = Node(package='rj_robocup',
                 executable='grSim',
                 arguments=['--headless'])

    soccer = Node(package='rj_robocup',
                  executable='soccer',
                  output='screen',
                  arguments=['-b', '-sim', '-noref'],
                  on_exit=Shutdown())

    config_server = Node(package='rj_robocup',
                         executable='config_server',
                         output='screen',
                         on_exit=Shutdown())

    vision_receiver_launch_path = str(launch_dir / "vision_receiver.launch.py")
    vision_receiver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(vision_receiver_launch_path))

    ref_receiver = Node(package='rj_robocup',
                        executable='internal_referee_node',
                        output='screen',
                        on_exit=Shutdown())

    vision_filter_launch_path = str(launch_dir / "vision_filter.launch.py")
    vision_filter = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(vision_filter_launch_path))

    return LaunchDescription([
        stdout_linebuf_envvar, config_server, soccer, grsim, vision_receiver,
        vision_filter, ref_receiver
    ])
