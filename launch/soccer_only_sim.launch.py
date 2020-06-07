import launch
from launch import LaunchDescription

from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package="rj_robocup",
             node_executable="soccer",
             output="screen",
             arguments=["-sim", "-pbk", "testing.pbk"],
             on_exit=launch.actions.Shutdown())
    ])
