import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    Shutdown,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():
    bringup_dir = Path(get_package_share_directory("rj_robocup"))
    launch_dir = bringup_dir / "launch"

    team_flag = LaunchConfiguration("team_flag", default="-b")
    use_grsim = LaunchConfiguration("use_grsim", default="True")
    ref_flag = LaunchConfiguration("ref_flag", default="-noref")
    headless_flag = LaunchConfiguration("headless_flag", default="")
    radio_flag = LaunchConfiguration("radio_flag", default="sim_radio_node")
    ref_rec_flag = LaunchConfiguration("ref_rec_flag", default="internal_referee_node")

    stdout_linebuf_envvar = SetEnvironmentVariable(
        "RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED", "1"
    )

    grsim = Node(
        condition=IfCondition(PythonExpression([use_grsim])),
        package="rj_robocup",
        executable="grSim",
        arguments=[headless_flag],
        on_exit=Shutdown(),
    )

    soccer_launch_path = str(launch_dir / "soccer.launch.py")
    soccer = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(soccer_launch_path),
        launch_arguments={
            "team_flag": team_flag,
            "sim_flag": "-sim",
            "ref_flag": ref_flag,
            "radio_flag": radio_flag,
            "ref_rec_flag": ref_rec_flag,
        }.items(),
    )

    return LaunchDescription(
        [
            stdout_linebuf_envvar,
            DeclareLaunchArgument("team_flag", default_value="-y"),
            DeclareLaunchArgument("ref_flag", default_value="-noref"),
            DeclareLaunchArgument("headless_flag", default_value=""),
            DeclareLaunchArgument("direction_flag", default_value="plus"),
            DeclareLaunchArgument("radio_flag", default_value="sim_radio_node"),
            DeclareLaunchArgument(
                "ref_rec_flag", default_value="internal_referee_node"
            ),
            stdout_linebuf_envvar,
            grsim,
            soccer,
        ]
    )
