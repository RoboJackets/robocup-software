import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    Shutdown,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("rj_robocup"), "config", "sim.yaml"
    )
    bringup_dir = Path(get_package_share_directory("rj_robocup"))
    launch_dir = bringup_dir / "launch"

    stdout_linebuf_envvar = SetEnvironmentVariable(
        "RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED", "1"
    )

    grsim = Node(package="rj_robocup", executable="grSim", arguments=[])

    radio = Node(
        package="rj_radio",
        executable="sim_radio_node",
        output="screen",
        on_exit=Shutdown(),
    )

    control = Node(
        package="rj_control",
        executable="motion_control_node",
        output="screen",
        on_exit=Shutdown(),
    )

    config_server = Node(
        package="rj_config_server",
        executable="rj_config_server_node",
        output="screen",
        on_exit=Shutdown(),
    )

    vision_receiver = Node(
        package="rj_vision_receiver",
        executable="rj_vision_receiver_node",
        output="screen",
        parameters=[config],
        on_exit=Shutdown(),
    )

    ref_receiver = Node(
        package="rj_referee",
        executable="internal_referee_node",
        output="screen",
        on_exit=Shutdown(),
    )

    vision_filter = Node(
        package="rj_vision_filter",
        executable="rj_vision_filter_node",
        output="screen",
        parameters=[config],
        on_exit=Shutdown(),
    )

    return LaunchDescription(
        [
            grsim,
            stdout_linebuf_envvar,
            config_server,
            radio,
            control,
            vision_receiver,
            vision_filter,
            ref_receiver,
        ]
    )
