import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription

from launch.actions import (IncludeLaunchDescription, DeclareLaunchArgument,
                            GroupAction, Shutdown)
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory("rj_robocup")
    launch_dir = os.path.join(bringup_dir, "launch")

    # Create launch configuration variables
    sim = LaunchConfiguration("sim")
    team = LaunchConfiguration("team")
    defend_plus = LaunchConfiguration("defend_plus")

    # Declare Launch arguments
    declare_sim = DeclareLaunchArgument(
        "sim",
        default_value="True",
        description="Whether to pass -sim to soccer or not. Default True")
    declare_team = DeclareLaunchArgument(
        "team",
        default_value="b",
        description="What team to play as [y|b]. Default b")
    declare_defend_plus = DeclareLaunchArgument(
        "defend_plus",
        default_value="False",
        description="Whether to defend plus. Default False.")

    config_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, "config_server.launch.py")))

    vision_receiver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, "vision_receiver.launch.py")))

    sim_arg = PythonExpression(['"-sim" if ', sim, ' else "" '])
    team_arg = PythonExpression(['"-" + "', team, '"'])
    defend_plus_arg_1 = PythonExpression(
        ['"-defend" if ', defend_plus, ' else "" '])
    defend_plus_arg_2 = PythonExpression(
        ['"plus" if ', defend_plus, ' else "" '])

    soccer = Node(
        package="rj_robocup",
        node_executable="soccer",
        output="screen",
        arguments=[sim_arg, team_arg, defend_plus_arg_1, defend_plus_arg_2],
        on_exit=Shutdown())

    launch_nodes = GroupAction([config_server, vision_receiver, soccer])

    return LaunchDescription(
        [declare_sim, declare_team, declare_defend_plus, launch_nodes])
