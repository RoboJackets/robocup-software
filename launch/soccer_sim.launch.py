import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    bringup_dir = get_package_share_directory('rj_robocup')
    launch_dir = os.path.join(bringup_dir, 'launch')

    config_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, "config_server.launch.py")))

    vision_receiver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, "vision_receiver.launch.py")))

    soccer = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, "soccer_only_sim.launch.py")))

    return LaunchDescription([
        soccer,
        config_server,
        vision_receiver
    ])
