import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="rj_base_station",
            namespace="radio_0",
            executable="radio_node",
            name="radio_0",
            parameters=[os.path.join(
                get_package_share_directory("rj_base_station"),
                "config",
                "radios.yaml"
            )],
            respawn=False
        ),
        Node(
            package="rj_base_station",
            namespace="radio_1",
            executable="radio_node",
            name="radio_1",
            parameters=[os.path.join(
                get_package_share_directory("rj_base_station"),
                "config",
                "radios.yaml"
            )],
            respawn=False
        ),
        Node(
            package="rj_base_station",
            namespace="",
            executable="alive_robots_node",
            name="alive_robots_node",
            respawn=False
        )
    ])