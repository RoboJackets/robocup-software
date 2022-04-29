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
    config_yaml = LaunchConfiguration("config_yaml", default="sim.yaml")

    config = os.path.join(
        get_package_share_directory("rj_robocup"), "config", str(config_yaml)
    )
    bringup_dir = Path(get_package_share_directory("rj_robocup"))
    launch_dir = bringup_dir / "launch"

    use_internal_ref = LaunchConfiguration("use_internal_ref", default="True")
    use_sim_radio = LaunchConfiguration("use_sim_radio", default="True")
    team_flag = LaunchConfiguration("team_flag", default="-b")
    sim_flag = LaunchConfiguration("sim_flag", default="-sim")
    ref_flag = LaunchConfiguration("ref_flag", default="-noref")
    direction_flag = LaunchConfiguration("direction_flag", default="plus")

    stdout_linebuf_envvar = SetEnvironmentVariable(
        "RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED", "1"
    )

    soccer = Node(
        package="rj_robocup",
        executable="soccer",
        output="screen",
        arguments=[team_flag, sim_flag, ref_flag, "-defend", direction_flag],
        parameters=[config],
        on_exit=Shutdown(),
    )

    config_server = Node(
        package="rj_robocup",
        executable="config_server",
        output="screen",
        arguments=[team_flag, sim_flag, ref_flag, "-defend", direction_flag],
        parameters=[config],
        on_exit=Shutdown(),
    )

    sim_radio = Node(
        condition=IfCondition(PythonExpression([use_sim_radio])),
        package="rj_robocup",
        executable="sim_radio_node",
        output="screen",
        parameters=[config],
        on_exit=Shutdown(),
    )

    network_radio = Node(
        condition=IfCondition(PythonExpression([str(not use_sim_radio)])),
        package="rj_robocup",
        executable="network_radio_node",
        output="screen",
        parameters=[config],
        on_exit=Shutdown(),
    )

    control = Node(
        package="rj_robocup",
        executable="control_node",
        output="screen",
        parameters=[config],
        on_exit=Shutdown(),
    )

    planner = Node(
        package="rj_robocup",
        executable="planner_node",
        output="screen",
        parameters=[config],
        on_exit=Shutdown(),
    )

    gameplay = Node(
        package="rj_robocup",
        executable="gameplay_node",
        output="screen",
        parameters=[config],
        emulate_tty=True,
        on_exit=Shutdown(),
    )

    vision_receiver = Node(
        package="rj_robocup",
        executable="vision_receiver",
        output="screen",
        parameters=[config],
        on_exit=Shutdown(),
    )

    internal_ref_receiver = Node(
        condition=IfCondition(PythonExpression([use_internal_ref])),
        package="rj_robocup",
        executable="internal_referee_node",
        output="screen",
        parameters=[config],
        on_exit=Shutdown(),
    )

    external_ref_receiver = Node(
        condition=IfCondition(PythonExpression([str(not use_internal_ref)])),
        package="rj_robocup",
        executable="external_referee_node",
        output="screen",
        parameters=[config],
        on_exit=Shutdown(),
    )

    vision_filter = Node(
        package="rj_robocup",
        executable="rj_vision_filter",
        output="screen",
        parameters=[config],
        on_exit=Shutdown(),
    )

    global_param_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(launch_dir / "global_param_server.launch.py"))
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("team_flag", default_value=""),
            DeclareLaunchArgument("sim_flag", default_value=""),
            DeclareLaunchArgument("ref_flag", default_value=""),
            DeclareLaunchArgument("direction_flag", default_value="plus"),
            stdout_linebuf_envvar,
            config_server,
            global_param_server,
            soccer,
            sim_radio,
            network_radio,
            control,
            planner,
            vision_receiver,
            vision_filter,
            internal_ref_receiver,
            external_ref_receiver,
            gameplay,
        ]
    )
