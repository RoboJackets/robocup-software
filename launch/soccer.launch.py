import os
import sys
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

from launch import LaunchContext, LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    SetLaunchConfiguration,
    Shutdown,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PythonExpression,
    TextSubstitution,
)


def generate_launch_description():
    bringup_dir = Path(get_package_share_directory("rj_robocup"))
    launch_dir = bringup_dir / "launch"

    # there must be duplicate defaults in LaunchConfiguration and in DeclareLaunchArgument
    #
    # https://answers.ros.org/question/322874/ros2-what-is-different-between-declarelaunchargument-and-launchconfiguration/
    use_manual_control = LaunchConfiguration("use_manual_control", default="False")

    team_flag = LaunchConfiguration("team_flag", default="-b")
    direction_flag = LaunchConfiguration("direction_flag", default="plus")

    run_sim = LaunchConfiguration("run_sim", default="True")
    # TODO: figure out what the hell sim_flag does
    sim_flag = LaunchConfiguration("sim_flag", default="-sim")

    use_internal_ref = LaunchConfiguration("use_internal_ref", default="True")
    ref_flag = LaunchConfiguration("ref_flag", default="-noref")

    # args that can be set from cmd line
    # https://docs.ros.org/en/foxy/How-To-Guides/Launch-file-different-formats.html
    # TODO: match above args to this style of declaring args, since this is cleaner
    # or smth like:
    # server_port_la = DeclareLaunchArgument(...)
    # server_port_lc = LaunchConfiguration(...)
    # so it's clearer what should match

    # output port from field comp's perspective (NetworkRadio only)
    # IP address will auto-latch to Ubiquiti cloud key's IP per UDP v4 protocol
    server_port = DeclareLaunchArgument(
        "server_port", default_value=TextSubstitution(text="25565")
    )

    param_config = LaunchConfiguration("param_config")
    param_config_filepath = LaunchConfiguration("param_config_filepath")

    stdout_linebuf_envvar = SetEnvironmentVariable(
        "RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED", "1"
    )

    global_param_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(launch_dir / "global_param_server.launch.py"))
    )

    return LaunchDescription(
        [
            server_port,
            DeclareLaunchArgument("team_flag", default_value="-y"),
            DeclareLaunchArgument("ref_flag", default_value="-noref"),
            DeclareLaunchArgument("direction_flag", default_value="plus"),
            DeclareLaunchArgument("use_internal_ref", default_value="True"),
            DeclareLaunchArgument("run_sim", default_value="True"),
            DeclareLaunchArgument("param_config", default_value="sim_params.yaml"),
            DeclareLaunchArgument(
                "param_config_filepath",
                default_value=[
                    TextSubstitution(
                        text=os.path.join(
                            get_package_share_directory("rj_robocup"), "config", ""
                        )
                    ),
                    param_config,
                ],
            ),
            DeclareLaunchArgument("use_manual_control", default_value="False"),
            DeclareLaunchArgument("use_sim_radio", default_value="True"),
            stdout_linebuf_envvar,
            Node(
                package="rj_robocup",
                executable="config_server",
                output="screen",
                arguments=[team_flag, sim_flag, ref_flag, "-defend", direction_flag],
                parameters=[param_config_filepath],
                on_exit=Shutdown(),
            ),
            global_param_server,
            Node(
                package="rj_robocup",
                executable="soccer",
                output="screen",
                arguments=[team_flag, sim_flag, ref_flag, "-defend", direction_flag],
                parameters=[param_config_filepath],
                on_exit=Shutdown(),
            ),
            Node(
                condition=IfCondition(PythonExpression([run_sim])),
                package="rj_robocup",
                executable="sim_radio_node",
                output="screen",
                parameters=[param_config_filepath],
                on_exit=Shutdown(),
            ),
            Node(
                condition=IfCondition(PythonExpression(["not ", run_sim])),
                package="rj_robocup",
                executable="network_radio_node",
                output="screen",
                parameters=[
                    param_config_filepath,
                    {"server_port": LaunchConfiguration("server_port")},
                ],
                on_exit=Shutdown(),
            ),
            Node(
                package="rj_robocup",
                executable="control_node",
                output="screen",
                parameters=[param_config_filepath],
                on_exit=Shutdown(),
            ),
            Node(
                package="rj_robocup",
                executable="planner_node",
                output="screen",
                parameters=[param_config_filepath],
                on_exit=Shutdown(),
            ),
            # spawn manual node only if use_manual_control is True
            Node(
                condition=IfCondition(PythonExpression([use_manual_control])),
                package="rj_robocup",
                executable="manual_control_node",
                output="screen",
                on_exit=Shutdown(),
            ),
            # spawn gameplay only if manual is not on
            Node(
                condition=IfCondition(PythonExpression(["not ", use_manual_control])),
                package="rj_robocup",
                executable="gameplay_node",
                output="screen",
                parameters=[param_config_filepath],
                emulate_tty=True,
                on_exit=Shutdown(),
            ),
            Node(
                package="rj_robocup",
                executable="vision_receiver",
                output="screen",
                parameters=[param_config_filepath],
                on_exit=Shutdown(),
            ),
            Node(
                condition=IfCondition(PythonExpression([use_internal_ref])),
                package="rj_robocup",
                executable="internal_referee_node",
                output="screen",
                parameters=[param_config_filepath],
                on_exit=Shutdown(),
            ),
            Node(
                condition=IfCondition(PythonExpression(["not ", use_internal_ref])),
                package="rj_robocup",
                executable="external_referee_node",
                output="screen",
                parameters=[param_config_filepath],
                on_exit=Shutdown(),
            ),
            Node(
                package="rj_robocup",
                executable="rj_vision_filter",
                output="screen",
                parameters=[param_config_filepath],
                on_exit=Shutdown(),
            ),
        ]
    )
