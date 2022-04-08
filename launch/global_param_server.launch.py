import os
from ament_index_python.packages import get_package_share_directory
import launch
from launch import LaunchDescription

import launch_ros.actions


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("rj_robocup"), "config", "real.yaml"
    )
    return LaunchDescription(
        [
            launch_ros.actions.Node(
                package="rj_robocup",
                executable="global_param_server_node",
                output="screen",
                on_exit=launch.actions.Shutdown(),
                parameters=[config],
            )
        ]
    )
