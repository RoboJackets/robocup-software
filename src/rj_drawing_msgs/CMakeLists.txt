# ======================================================================
# Preamble
# ======================================================================
cmake_minimum_required(VERSION 3.16)
project(rj_drawing_msgs)

# ======================================================================
# Find package
# ======================================================================
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rj_geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

# ======================================================================
# Define Targets
# ======================================================================
# This creates the target rj_drawing_msgs__rosidl_typesupport_cpp
rosidl_generate_interfaces(
  rj_drawing_msgs
  # Messages
  msg/DebugDraw.msg
  msg/DrawColor.msg
  msg/DrawShapes.msg
  msg/DrawSegment.msg
  msg/DrawPose.msg
  msg/DrawPath.msg
  msg/DrawText.msg

  DEPENDENCIES
  std_msgs
  builtin_interfaces
  rj_geometry_msgs)

# ======================================================================
# ROS2 packaging
# ======================================================================
ament_export_dependencies(rosidl_default_runtime)
ament_package()
