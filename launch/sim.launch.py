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
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    bringup_dir = Path(get_package_share_directory("rj_robocup"))
    launch_dir = bringup_dir / "launch"

    config_yaml = LaunchConfiguration("config_yaml", default="sim.yaml")
    use_internal_ref = LaunchConfiguration("use_internal_ref", default="True")
    use_sim_radio = LaunchConfiguration("use_sim_radio", default="True")
    team_flag = LaunchConfiguration("team_flag", default="-b")
    ref_flag = LaunchConfiguration("ref_flag", default="-noref")
    direction_flag = LaunchConfiguration("direction_flag", default="plus")

    stdout_linebuf_envvar = SetEnvironmentVariable(
        "RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED", "1"
    )

    soccer_launch_path = str(launch_dir / "soccer.launch.py")
    soccer = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(soccer_launch_path),
        launch_arguments={
            "team_flag": team_flag,
            "sim_flag": "-sim",
            "ref_flag": ref_flag,
            "direction_flag": direction_flag,
            "use_sim_radio": use_sim_radio,
            "use_internal_ref": use_internal_ref,
            "config_yaml": config_yaml,
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("team_flag", default_value="-y"),
            DeclareLaunchArgument("ref_flag", default_value="-noref"),
            DeclareLaunchArgument("direction_flag", default_value="plus"),
            DeclareLaunchArgument("use_internal_ref", default_value="True"),
            DeclareLaunchArgument("use_sim_radio", default_value="True"),
            DeclareLaunchArgument("config_yaml", default_value="sim.yaml"),
            stdout_linebuf_envvar,
            soccer,
        ]
    )
