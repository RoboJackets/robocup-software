from launch import LaunchDescription

from launch_ros.actions import Node
from launch.actions import Shutdown


def generate_launch_description():
    publish_hz = 120.0

    return LaunchDescription([
        Node(package="rj_robocup",
             executable="rj_vision_filter",
             output="screen",
             parameters=[{
                 "publish_hz": publish_hz
             }],
             on_exit=Shutdown(),
        )
    ])
