import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription

from launch_ros.actions import Node
from launch.actions import Shutdown


def generate_launch_description():
<<<<<<< HEAD
    sim_vision_port = 10006
    shared_vision_port_single_primary = 10002

    hz = 120.0
    port = sim_vision_port
=======
    config = os.path.join(get_package_share_directory('rj_robocup'), 'config',
                          'sim.yaml')
>>>>>>> 19154d927b2b8fa91e7749eb7539d8edf2f284b0

    return LaunchDescription([
        Node(
            package="rj_robocup",
            executable="vision_receiver",
            output="screen",
            parameters=[config],
            on_exit=Shutdown(),
        )
    ])
