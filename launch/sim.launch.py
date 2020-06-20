import launch
from launch import LaunchDescription

import launch_ros.actions


def generate_launch_description():
    return LaunchDescription([
        launch_ros.actions.Node(package='rj_robocup',
                                executable='grSim',
                                arguments=['--headless']),
        launch_ros.actions.Node(package='rj_robocup',
                                executable='soccer',
                                output='screen',
                                arguments=['-b', '-sim'],
                                on_exit=launch.actions.Shutdown()),
    ])
