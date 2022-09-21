import os
import sys
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node, PushRosNamespace

from launch import LaunchContext, LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    SetLaunchConfiguration,
    Shutdown,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():
    bringup_dir = Path(get_package_share_directory("rj_robocup"))
    launch_dir = bringup_dir / "launch"

    stdout_linebuf_envvar = SetEnvironmentVariable(
        "RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED", "1"
    )

    ref_flag = LaunchConfiguration("ref_flag", default="-noref")
    use_internal_ref = LaunchConfiguration("use_internal_ref", default="True")
    run_sim = LaunchConfiguration("run_sim", default="False")
    use_sim_radio = LaunchConfiguration("use_sim_radio", default="True")

    soccer_launch_path = str(launch_dir / "soccer.launch.py")

    config_yaml = "real_params.yaml"

    net_yaml = "real_network_params.yaml"

    alt_net_yaml = "alt_real_network_params.yaml"

    soccer_yellow = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(soccer_launch_path),
        launch_arguments={
            "main_config_yaml": config_yaml,
            "network_config_yaml": net_yaml,
            "team_flag": "-y",
            "sim_flag": "-sim",
            "ref_flag": ref_flag,
            "direction_flag": "plus",
            "use_internal_ref": use_internal_ref,
            "run_sim": run_sim,
            "use_sim_radio": use_sim_radio,
        }.items(),
    )

    yellow = GroupAction([PushRosNamespace("yellow"), soccer_yellow])

    soccer_blue = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(soccer_launch_path),
        launch_arguments={
            "main_config_yaml": config_yaml,
            "network_config_yaml": alt_net_yaml,
            "team_flag": "-b",
            "sim_flag": "-sim",
            "ref_flag": ref_flag,
            "direction_flag": "minus",
            "use_internal_ref": use_internal_ref,
            "run_sim": run_sim,
            "use_sim_radio": use_sim_radio,
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
