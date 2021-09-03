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
    sim_flag = LaunchConfiguration('sim_flag', default='-sim')
    ref_flag = LaunchConfiguration('ref_flag', default='-noref')
    headless_flag = LaunchConfiguration('headless_flag', default='')
    direction_flag = LaunchConfiguration('direction_flag', default='plus')

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED', '1')

    soccer = Node(
        package='rj_robocup',
        executable='soccer',
        output='screen',
        arguments=[team_flag, sim_flag, ref_flag, '-defend', direction_flag],
        on_exit=Shutdown())

    config_server = Node(
        package='rj_robocup',
        executable='config_server',
        output='screen',
        arguments=[team_flag, sim_flag, ref_flag, '-defend', direction_flag],
        on_exit=Shutdown())

    radio = Node(package='rj_robocup',
                 executable='network_radio_node',
                 output='screen',
                 on_exit=Shutdown())

    manual = Node(package='rj_robocup',
                  executable='manual_control_node',
                  output='screen',
                  on_exit=Shutdown())

    grsim = Node(package='rj_robocup',
                 executable='grSim',
                 arguments=[headless_flag])

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
        DeclareLaunchArgument('team_flag', default_value=''),
        DeclareLaunchArgument('sim_flag', default_value='-sim'),
        DeclareLaunchArgument('ref_flag', default_value=''),
        DeclareLaunchArgument('direction_flag', default_value='plus'),
        stdout_linebuf_envvar,
        config_server,
        soccer,
        radio,
        manual,
        vision_receiver,
        vision_filter,
        ref_receiver,
        # grsim,
    ])
