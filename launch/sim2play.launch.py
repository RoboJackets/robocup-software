import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node, PushRosNamespace

from launch import LaunchDescription
from launch.actions import (
    GroupAction,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    bringup_dir = Path(get_package_share_directory("rj_robocup"))
    launch_dir = bringup_dir / "launch"

    stdout_linebuf_envvar = SetEnvironmentVariable(
        "RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED", "1"
    )

    ref_flag = LaunchConfiguration("ref_flag", default="-noref")
    # headless_flag = LaunchConfiguration("headless_flag", default="")

    soccer_launch_path = str(launch_dir / "soccer.launch.py")

    soccer_yellow = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(soccer_launch_path),
        launch_arguments={
            "team_flag": "-y",
            "sim_flag": "-sim",
            "ref_flag": ref_flag,
            "direction_flag": "plus",
        }.items(),
    )

    yellow = GroupAction([PushRosNamespace("yellow"), soccer_yellow])

    soccer_blue = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(soccer_launch_path),
        launch_arguments={
            "team_flag": "-b",
            "sim_flag": "-sim",
            "ref_flag": ref_flag,
            "direction_flag": "minus",
        }.items(),
    )

    blue = GroupAction([PushRosNamespace("blue"), soccer_blue])

    return LaunchDescription([stdout_linebuf_envvar, yellow, blue])
