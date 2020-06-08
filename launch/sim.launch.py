import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription

from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():
    bringup_dir = get_package_share_directory('rj_robocup')
    launch_dir = os.path.join(bringup_dir, 'launch')

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED', '1')

    soccer_sim = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        os.path.join(launch_dir, "soccer_sim.launch.py")),
                                          launch_arguments={
                                              "sim": "True",
                                              "team": "b",
                                              "defend_plus": "False",
                                          }.items())

    grsim = Node(package="rj_robocup",
                 node_executable="grSim",
                 arguments=["--headless"])

    return LaunchDescription([stdout_linebuf_envvar, soccer_sim, grsim])
