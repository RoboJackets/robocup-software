cmake_minimum_required(VERSION 3.16)
project(rj_topic_utils)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# Declare header-only library
add_library(rj_topic_utils INTERFACE)
target_include_directories(rj_topic_utils INTERFACE include)
ament_target_dependencies(rj_topic_utils INTERFACE rclcpp)

ament_export_dependencies(rclcpp)

# Install headers
install(DIRECTORY include/ DESTINATION include/)

# Install the target
install(
  TARGETS rj_topic_utils
  INCLUDES DESTINATION include
)

ament_package()
