# ======================================================================
# Preamble
# ======================================================================
cmake_minimum_required(VERSION 3.16)
project(rj_geometry_msgs)

# ======================================================================
# Find package
# ======================================================================
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

# ======================================================================
# Define Targets
# ======================================================================
# This creates the target rj_geometry_msgs__rosidl_typesupport_cpp
rosidl_generate_interfaces(
  rj_geometry_msgs
  msg/Circle.msg
  msg/Line.msg
  msg/Point.msg
  msg/Polygon.msg
  msg/Pose.msg
  msg/Rect.msg
  msg/Segment.msg
  msg/ShapeSet.msg
  msg/Twist.msg
  DEPENDENCIES
  std_msgs)

# ======================================================================
# ROS2 packaging
# ======================================================================
ament_export_dependencies(rosidl_default_runtime)
ament_package()
