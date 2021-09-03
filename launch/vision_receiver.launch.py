import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription

from launch_ros.actions import Node
from launch.actions import Shutdown


def generate_launch_description():
    config = os.path.join(get_package_share_directory('rj_robocup'), 'config', 'field_b.yaml')

    sim_vision_port = 10006
    shared_vision_port_single_primary = 10002

    hz = 120.0
    port = sim_vision_port

    return LaunchDescription([
        Node(package="rj_robocup",
             executable="vision_receiver",
             output="screen",
             parameters=[config],
             on_exit=Shutdown(),
        )
    ])
