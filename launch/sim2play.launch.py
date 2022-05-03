import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node, PushRosNamespace

from launch import LaunchDescription
from launch.actions import (
    GroupAction,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
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
    use_internal_ref = LaunchConfiguration("use_internal_ref", default="True")
    use_sim_radio = LaunchConfiguration("use_sim_radio", default="True")
    config_yaml = LaunchConfiguration("config_yaml", default="sim.yaml")

    soccer_launch_path = str(launch_dir / "soccer.launch.py")

    soccer_yellow = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(soccer_launch_path),
        launch_arguments={
            "team_flag": "-y",
            "sim_flag": "-sim",
            "ref_flag": ref_flag,
            "direction_flag": "plus",
            "use_internal_ref": use_internal_ref,
            "use_sim_radio": use_sim_radio,
            "config_yaml": config_yaml,
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
            "use_internal_ref": use_internal_ref,
            "use_sim_radio": use_sim_radio,
            "config_yaml": config_yaml,
        }.items(),
    )

    blue = GroupAction([PushRosNamespace("blue"), soccer_blue])

    """
    Force blue to wait an arbitrary number of seconds,
    since there is a race condition in sim radio node.

    If problems re-emerge, implement better solution:
    - using boost inter-process locks in that node
    - or alternatively use some sort of launch file action
    that starts blue as soon as yellow done launching.
    """
    return LaunchDescription(
        [
            stdout_linebuf_envvar,
            yellow,
            TimerAction(period=3.0, actions=[blue]),
        ]
    )
