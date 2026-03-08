#
# Launch the line test node stack to test and tune PID for the simulator
#

import os
import pathlib

from typing import Optional

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import (
    DeclareLaunchArgument,
    SetEnvironmentVariable,
    Shutdown,
    ExecuteProcess
)
from launch.substitutions import (
    LaunchConfiguration,
    TextSubstitution
)

def find_simulator_cli() -> Optional[str]:
    for root, _, filenames in os.walk(pathlib.Path("~").expanduser()):
        if "simulator-cli" in filenames:
            return os.path.join(root, "simulator-cli")
    return None

def generate_launch_description():
    """
    Generate the launch description to run line test in the simulator
    """

    simulator_binary = find_simulator_cli()
    if simulator_binary is None:
        raise RuntimeError("simulator-cli could not be found... Please ensure it is downloaded first")
    
    rj_control_share = get_package_share_directory("rj_control")
    
    team_flag = LaunchConfiguration("team_flag")
    direction_flag = LaunchConfiguration("direction_flag")

    param_config = LaunchConfiguration("param_config")
    param_config_filepath = LaunchConfiguration("param_config_filepath")

    launch_description = [
        DeclareLaunchArgument(
            "server_port", default_value=TextSubstitution(text="25565")
        ),
        DeclareLaunchArgument(
            "team_name", default_value=TextSubstitution(text="RoboJackets")
        ),
        DeclareLaunchArgument("team_flag", default_value="-y"),
        DeclareLaunchArgument("direction_flag", default_value="plus"),
        DeclareLaunchArgument("param_config", default_value="sim_params.yaml"),
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
        SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "1"),
        ExecuteProcess(
            cmd=[simulator_binary, "--localhost", "-g", "2020B", "--realism", "RC2021"],
            output="screen",
            on_exit=Shutdown()
        ),
        Node(
            package="rj_vision_receiver",
            executable="rj_vision_receiver_node",
            output="screen",
            parameters=[os.path.join(
                get_package_share_directory("rj_vision_receiver"),
                "config",
                "sim_params.yaml"
            )],
            on_exit=Shutdown()
        ),
        Node(
            package="rj_config_server",
            executable="rj_config_server_node",
            output="screen",
            arguments=[team_flag, "-sim", "-noref", "-defend", direction_flag],
            parameters=[param_config_filepath],
            on_exit=Shutdown()
        ),
        Node(
            package="rj_param_utils",
            executable="global_param_server_node",
            output="screen",
            parameters=[
                os.path.join(
                    get_package_share_directory("rj_param_utils"),
                        "config",
                        "sim_params.yaml"
                )
            ],
            on_exit=Shutdown()
        ),
        Node(
            package="rj_radio",
            executable="sim_radio_node",
            output="screen",
            parameters=[param_config_filepath],
            on_exit=Shutdown()
        ),
        Node(
            package="rj_referee",
            executable="internal_referee_node",
            output="screen",
            parameters=[os.path.join(
                get_package_share_directory("rj_referee"),
                "config",
                "sim_params.yaml"
            )],
            on_exit=Shutdown()
        ),
        Node(
            package="rj_vision_filter",
            executable="rj_vision_filter_node",
            output="screen",
            parameters=[os.path.join(
                get_package_share_directory("rj_vision_filter"),
                "config",
                "sim_params.yaml"
            )],
            on_exit=Shutdown()
        ),
        Node(
            package="rosbridge_server",
            name="rosbridge",
            executable="rosbridge_websocket.py",
            on_exit=Shutdown()
        ),
        Node(
            package="rosapi",
            name="rosapi",
            executable="rosapi_node",
            on_exit=Shutdown()
        ),
        ExecuteProcess(
            cmd=["ros2", "run", "rj_ui", "rj_ui"],
            on_exit=Shutdown()
        )
    ]

    launch_description.append(Node(
        package="rj_control",
        namespace="robot_0",
        executable="control_node",
        parameters=[os.path.join(
            rj_control_share,
            "config",
            "sim_params.yaml"
        )],
        output="screen",
        on_exit=Shutdown()
    ))
    launch_description.append(Node(
        package="rj_testing",
        namespace="robot_0",
        executable="line_test_node",
        output="screen",
        on_exit=Shutdown()
    ))

    # for robot_id in range(6):
    #     launch_description.append(Node(
    #         package="rj_control",
    #         namespace=f"robot_{robot_id}",
    #         executable="control_node",
    #         parameters=[os.path.join(
    #             rj_control_share,
    #             "config",
    #             "sim_params.yaml"
    #         )],
    #         output="screen",
    #         on_exit=Shutdown()
    #     ))
    #     launch_description.append(Node(
    #         package="rj_testing",
    #         namespace=f"robot_{robot_id}",
    #         executable="line_test_node",
    #         output="screen",
    #         on_exit=Shutdown()
    #     ))

    return LaunchDescription(launch_description)


