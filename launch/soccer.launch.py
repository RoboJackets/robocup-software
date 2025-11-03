import os
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
    """
    Launch files, though written in Python, are not normal Python scripts.

    Think of them as .yaml files that happen to use Python syntax. Since they
    are statically built before being used, they are unable to log debug info
    or use control-flow constructs like if-else.

    https://docs.ros.org/en/foxy/How-To-Guides/Launch-file-different-formats.html
    """

    # LaunchConfiguration objects are like uninitialized variables, e.g.
    #  > bool run_sim;
    # to be later filled in by a LaunchArgument

    # output port from field comp's perspective (NetworkRadio only)
    # IP address will auto-latch to Ubiquiti cloud key's IP per UDP v4 protocol
    LaunchConfiguration("server_port")
    LaunchConfiguration("team_name")

    use_manual_control = LaunchConfiguration("use_manual_control")

    team_flag = LaunchConfiguration("team_flag")
    direction_flag = LaunchConfiguration("direction_flag")

    run_sim = LaunchConfiguration("run_sim")
    sim_flag = LaunchConfiguration("sim_flag")
    run_line_test = LaunchConfiguration("run_line_test")

    use_internal_ref = LaunchConfiguration("use_internal_ref")
    ref_flag = LaunchConfiguration("ref_flag")

    param_config = LaunchConfiguration("param_config")
    param_config_filepath = LaunchConfiguration("param_config_filepath")

    # make debug text buffer to console
    stdout_linebuf_envvar = SetEnvironmentVariable(
        "RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED", "1"
    )

    return LaunchDescription(
        [
            # LaunchArguments are declared here to be filled in by CLI arg,
            # e.g.
            #  > ros2 launch rj_robocup soccer.launch.py run_sim:=True direction_flag:=plus
            # will launch soccer with run_sim=True, direction_flag=+x
            DeclareLaunchArgument(
                "server_port", default_value=TextSubstitution(text="25565")
            ),
            DeclareLaunchArgument(
                "team_name", default_value=TextSubstitution(text="RoboJackets")
            ),
            DeclareLaunchArgument("team_flag", default_value="-y"),
            DeclareLaunchArgument("ref_flag", default_value="-noref"),
            DeclareLaunchArgument("direction_flag", default_value="plus"),
            DeclareLaunchArgument("use_internal_ref", default_value="True"),
            DeclareLaunchArgument("run_sim", default_value="True"),
            DeclareLaunchArgument("sim_flag", default_value="-sim"),
            DeclareLaunchArgument(
                "param_config",
                default_value=PythonExpression(
                    ["'sim_params.yaml' if ", run_sim, " else 'real_params.yaml'"]
                ),
            ),
            DeclareLaunchArgument("use_manual_control", default_value="False"),
            DeclareLaunchArgument("use_sim_radio", default_value="True"),
            # this launch arg shouldn't be used, is solely dependent on run_sim
            # (above, param_config is defined by run_sim)
            DeclareLaunchArgument(
                "param_config_filepath",
                default_value=[
                    TextSubstitution(
                        text=os.path.join(
                            get_package_share_directory("rj_param_utils"), "config", ""
                        )
                    ),
                    param_config,
                ],
            ),
            DeclareLaunchArgument("run_line_test", default_value="False"),
            stdout_linebuf_envvar,
            # Node spawns all of the ROS nodes, defined in main() of various
            # cpp files, e.g. vision_receiver.cpp, planner_node_main.cpp
            # Each of these take in the results of various
            # LaunchConfiguration/Arguments above, e.g. param_config_filepath
            #
            # Note the order doesn't matter here: ROS nodes launch in some
            # random order (there are Executors to change that)
            Node(
                package="rj_vision_receiver",
                executable="rj_vision_receiver_node",
                output="screen",
                parameters=[param_config_filepath],
                on_exit=Shutdown(),
            ),
            Node(
                package="rj_config_server",
                executable="rj_config_server_node",
                output="screen",
                arguments=[team_flag, sim_flag, ref_flag, "-defend", direction_flag],
                parameters=[param_config_filepath],
                on_exit=Shutdown(),
            ),
            Node(
                package="rj_param_utils",
                executable="global_param_server_node",
                output="screen",
                parameters=[
                    os.path.join(
                        get_package_share_directory("rj_param_utils"),
                        "config",
                        "sim.yaml"
                    )
                ],
                on_exit=Shutdown(),
            ),
            Node(
                package="rj_ui",
                executable="rj_ui_node",
                output="screen",
                arguments=[team_flag, sim_flag, ref_flag, "-defend", direction_flag],
                parameters=[param_config_filepath],
                on_exit=Shutdown(),
            ),
            Node(
                condition=IfCondition(PythonExpression([run_sim])),
                package="rj_radio",
                executable="sim_radio_node",
                output="screen",
                parameters=[param_config_filepath],
                on_exit=Shutdown(),
            ),
            Node(
                condition=IfCondition(PythonExpression([run_sim])),
                package="rj_radio",
                executable="soccer_mom_node",
                output="screen",
                parameters=[param_config_filepath],
                on_exit=Shutdown(),
            ),
            Node(
                condition=IfCondition(PythonExpression(["not ", run_sim])),
                package="rj_radio",
                executable="network_radio_node",
                output="screen",
                parameters=[
                    param_config_filepath,
                    {"server_port": LaunchConfiguration("server_port")},
                ],
                on_exit=Shutdown(),
            ),
            Node(
                condition=IfCondition(PythonExpression(["not ", use_manual_control])),
                package="rj_control",
                executable="motion_control_node",
                output="screen",
                parameters=[param_config_filepath],
                on_exit=Shutdown(),
            ),
            Node(
                package="rj_planning",
                executable="planner_node",
                output="screen",
                parameters=[param_config_filepath],
                on_exit=Shutdown(),
            ),
            # spawn manual node only if use_manual_control is True
            Node(
                condition=IfCondition(PythonExpression([use_manual_control])),
                package="rj_joystick",
                executable="manual_control_node",
                output="screen",
                on_exit=Shutdown(),
            ),
            # spawn internal_ref/external_ref based on internal_ref
            # LaunchArgument
            Node(
                condition=IfCondition(PythonExpression(["not ", use_manual_control, " and not ", run_line_test])),
                package="rj_strategy",
                executable="agent_action_clients",
                output="screen",
                parameters=[param_config_filepath],
                on_exit=Shutdown(),
            ),
            Node(
                condition=IfCondition(PythonExpression([run_line_test])),
                package="rj_strategy",
                executable="straight_line_test_node",
                output="screen",
                parameters=[param_config_filepath],
                on_exit=Shutdown(),
            ),
            Node(
                condition=IfCondition(PythonExpression([use_internal_ref])),
                package="rj_referee",
                executable="internal_referee_node",
                output="screen",
                parameters=[param_config_filepath],
                on_exit=Shutdown(),
            ),
            Node(
                condition=IfCondition(PythonExpression(["not ", use_internal_ref])),
                package="rj_referee",
                executable="external_referee_node",
                output="screen",
                parameters=[
                    param_config_filepath,
                    {"team_name": LaunchConfiguration("team_name")},
                ],
                on_exit=Shutdown(),
            ),
            Node(
                package="rj_vision_filter",
                executable="rj_vision_filter_node",
                output="screen",
                parameters=[param_config_filepath],
                on_exit=Shutdown(),
            ),
            Node(
                package="rj_strategy",
                executable="kicker_picker_node",
                output="screen",
                parameters=[param_config_filepath],
                on_exit=Shutdown(),
            )
        ]
    )