import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription

from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory('rj_robocup')
    launch_dir = os.path.join(bringup_dir, 'launch')

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED', '1')

    grsim = Node(package='rj_robocup',
                 executable='grSim',
                 arguments=['--headless'])
    soccer = Node(package='rj_robocup',
                  executable='soccer',
                  output='screen',
                  arguments=['-b', '-sim'],
                  on_exit=Shutdown())

    vision_receiver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, "vision_receiver.launch.py")))

    return LaunchDescription([
        stdout_linebuf_envvar,
        grsim,
        soccer,
        vision_receiver
    ])
