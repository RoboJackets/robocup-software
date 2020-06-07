import launch
from launch import LaunchDescription

import launch_ros.actions


def generate_launch_description():
    return LaunchDescription([
        launch_ros.actions.Node(package="rj_robocup",
                                node_executable="grSim",
                                arguments=["--headless"]),
        launch_ros.actions.Node(package="rj_robocup",
                                node_executable="soccer",
                                output="screen",
                                arguments=["-sim", "-pbk", "testing.pbk"],
                                on_exit=launch.actions.Shutdown()),
    ])
