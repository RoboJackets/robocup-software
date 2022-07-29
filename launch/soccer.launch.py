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
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():
    # see ros2/launch source code to see how this works
    # right now, it doesn't work, lc never adds config_yaml
    # thus it always returns default, see source code
    config_yaml = LaunchConfiguration("config_yaml", default="sim.yaml")
    lc = LaunchContext(noninteractive=True)
    slc = SetLaunchConfiguration(config_yaml, config_yaml)
    slc.execute(lc)

    config = os.path.join(
        get_package_share_directory("rj_robocup"),
        "config",
        (lc.perform_substitution(config_yaml)),
    )

    bringup_dir = Path(get_package_share_directory("rj_robocup"))
    launch_dir = bringup_dir / "launch"

    # there must be duplicate defaults in LaunchConfiguration and in DeclareLaunchArgument
    #
    # https://answers.ros.org/question/322874/ros2-what-is-different-between-declarelaunchargument-and-launchconfiguration/
    use_internal_ref = LaunchConfiguration("use_internal_ref", default="True")
    use_sim_radio = LaunchConfiguration("use_sim_radio", default="True")
    team_flag = LaunchConfiguration("team_flag", default="-b")
    sim_flag = LaunchConfiguration("sim_flag", default="-sim")
    ref_flag = LaunchConfiguration("ref_flag", default="-noref")
    direction_flag = LaunchConfiguration("direction_flag", default="plus")
    use_manual_control = LaunchConfiguration("use_manual_control", default="False")

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
        condition=IfCondition(PythonExpression(["not ", use_sim_radio])),
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

    # spawn manual node only if use_manual_control is True
    manual = Node(
        condition=IfCondition(PythonExpression([use_manual_control])),
        package="rj_robocup",
        executable="manual_control_node",
        output="screen",
        on_exit=Shutdown(),
    )

    # spawn gameplay only if manual is not on
    gameplay = Node(
        condition=IfCondition(PythonExpression(["not ", use_manual_control])),
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
        condition=IfCondition(PythonExpression(["not ", use_internal_ref])),
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
            DeclareLaunchArgument("team_flag", default_value="-y"),
            DeclareLaunchArgument("ref_flag", default_value="-noref"),
            DeclareLaunchArgument("direction_flag", default_value="plus"),
            DeclareLaunchArgument("config_yaml", default_value="sim.yaml"),
            stdout_linebuf_envvar,
            config_server,
            global_param_server,
            soccer,
            control,
            planner,
            vision_receiver,
            vision_filter,
            # nodes below this line are XOR based on the header launch arg
            DeclareLaunchArgument("use_sim_radio", default_value="True"),
            sim_radio,
            network_radio,
            DeclareLaunchArgument("use_internal_ref", default_value="True"),
            internal_ref_receiver,
            external_ref_receiver,
            DeclareLaunchArgument("use_manual_control", default_value="False"),
            gameplay,
            manual,
        ]
    )
