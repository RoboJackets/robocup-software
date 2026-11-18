from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    robot_id = LaunchConfiguration("robot_id")
    params_file = PathJoinSubstitution(
        [FindPackageShare("rj_joystick"), "config", "keyboard_params.yaml"]
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_id", default_value="0"),
            Node(
                package="rj_joystick",
                executable="keyboard_joystick_node",
                name="keyboard_joy",
                output="screen",
                parameters=[params_file, {"robot_id": robot_id}],
            )
        ]
    )
