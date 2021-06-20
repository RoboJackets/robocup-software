import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription

from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, Shutdown, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    bringup_dir = Path(get_package_share_directory('rj_robocup'))
    launch_dir = bringup_dir / 'launch'

    team_flag = LaunchConfiguration('team_flag', default='-b')
    ref_flag = LaunchConfiguration('ref_flag', default='-noref')
    headless_flag = LaunchConfiguration('headless_flag', default='')

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED', '1')

    grsim = Node(package='rj_robocup',
                 executable='grSim',
                 arguments=[headless_flag],
                 on_exit=Shutdown())

    soccer_launch_path = str(launch_dir / "soccer.launch.py")
    soccer = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(soccer_launch_path),
        launch_arguments={
            'team_flag': team_flag,
            'sim_flag': '-sim',
            'ref_flag': ref_flag
        }.items())

    return LaunchDescription([
        stdout_linebuf_envvar,
        DeclareLaunchArgument('team_flag', default_value='-y'),
        DeclareLaunchArgument('ref_flag', default_value='-noref'),
        DeclareLaunchArgument('headless_flag', default_value=''),
        DeclareLaunchArgument('direction_flag', default_value='plus'),
        stdout_linebuf_envvar, grsim, soccer
    ])
