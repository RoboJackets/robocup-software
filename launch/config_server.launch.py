import launch
from launch import LaunchDescription

import launch_ros.actions


def generate_launch_description():
    return LaunchDescription([
        launch_ros.actions.Node(
            package="rj_robocup",
            executable="config_server",
            output="screen",
            on_exit=launch.actions.Shutdown(),
        )
    ])
