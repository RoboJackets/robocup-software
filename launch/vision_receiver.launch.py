import launch
from launch import LaunchDescription

import launch_ros.actions


def generate_launch_description():
    sim_vision_port = 10020
    shared_vision_port_single_primary = 10002

    hz = 60.0
    port = sim_vision_port

    return LaunchDescription([
        launch_ros.actions.Node(package="rj_robocup",
                                node_executable="vision_receiver",
                                output="screen",
                                parameters=[{
                                    "hz": hz,
                                    "port": port
                                }])
    ])
