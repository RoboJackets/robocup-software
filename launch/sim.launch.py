#
# Launch our stack using the er-force simulator
#

import os

from utilities import find_simulator_cli

from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, Shutdown
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression, TextSubstitution

def generate_launch_description():
    """
    Generate the launch description to run our stack in the simulator
    """
    
    simulator_binary = find_simulator_cli()
    if simulator_binary is None:
        raise RuntimeError("simulator-cli could not be found... Please ensure it is downloaded first")
    
    LaunchConfiguration("server_port")
    LaunchConfiguration("team_name")

    use_manual_control = LaunchConfiguration("use_manual_control")
    team_flag = LaunchConfiguration("team_flag")
    direction_flag = LaunchConfiguration("direction_flag")
    run_line_test = LaunchConfiguration("run_line_test")
    use_internal_ref = LaunchConfiguration("use_internal_ref")
    ref_flag = LaunchConfiguration("ref_flag")
    param_config = LaunchConfiguration("param_config")
    param_config_filepath = LaunchConfiguration("param_config_filepath")

    # Make debug text buffer to console
    stdout_linebuf_envvar = SetEnvironmentVariable(
        "RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED", "1"
    )