from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression


def generate_launch_description():
    command = LaunchConfiguration("command")
    robot_id = LaunchConfiguration("robot_id")
    params_file = PathJoinSubstitution(
        [FindPackageShare("rj_joystick"), "config", "keyboard_params.yaml"]
    )

    def command_is(name):
        return IfCondition(
            PythonExpression(["'", command, "' == '", name, "'"])
        )

    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_id", default_value="0"),
            DeclareLaunchArgument("command", default_value="joystick"),
            Node(
                package="rj_joystick",
                executable="keyboard_joystick_node",
                name="keyboard_joy",
                output="screen",
                parameters=[params_file, {"robot_id": robot_id}],
                condition=command_is("keyboard"),
            ),
            Node(
                package="joy",
                executable="game_controller_node",
                name="game_controller",
                remappings=[
                    ("/joy", ["/joystick/joy/robot_", robot_id])
                ],
                condition=command_is("joystick"),
            ),
            Node(
                package="joy",
                executable="joy_enumerate_devices",
                condition=command_is("enumerate_devices"),
            ),
        ]
    )
