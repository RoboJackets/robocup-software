import os

import launch_ros.actions
from ament_index_python.packages import get_package_share_directory

import launch
from launch import LaunchDescription


def generate_launch_description():
    # TODO: make this align with soccer.launch.py (we never launch
    # global_param_server on its own)
    config = os.path.join(
        get_package_share_directory("rj_robocup"), "config", "sim.yaml"
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
