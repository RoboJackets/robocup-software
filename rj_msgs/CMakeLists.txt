#======================================================================
# Preamble
#======================================================================
cmake_minimum_required(VERSION 3.16)
project(rj_msgs)

#======================================================================
# Find package
#======================================================================
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rj_geometry_msgs REQUIRED)

#======================================================================
# Define Targets
#======================================================================
# This creates the target
# rj_geometry_msgs__rosidl_typesupport_cpp
rosidl_generate_interfaces(rj_msgs
    # Messages
    msg/BallState.msg
    msg/DetectionBall.msg
    msg/DetectionFrame.msg
    msg/DetectionRobot.msg
    msg/FieldDimensions.msg
    msg/GameSettings.msg
    msg/TeamColor.msg
    msg/RawProtobuf.msg
    msg/RobotState.msg
    msg/WorldState.msg
    # Services
    srv/SetFieldDimensions.srv
    srv/SetGameSettings.srv
    DEPENDENCIES std_msgs builtin_interfaces rj_geometry_msgs
    )

#======================================================================
# ROS2 packaging
#======================================================================
ament_export_dependencies(rosidl_default_runtime)
ament_package()
