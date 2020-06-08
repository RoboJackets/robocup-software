from launch import LaunchDescription

from launch_ros.actions import Node


def generate_launch_description():
    sim_vision_port = 10020
    shared_vision_port_single_primary = 10002

    hz = 120.0
    port = sim_vision_port

    return LaunchDescription([
        Node(package="rj_robocup",
             node_executable="vision_receiver",
             output="screen",
             parameters=[{
                 "hz": hz,
                 "port": port
             }])
    ])
