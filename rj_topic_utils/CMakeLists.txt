# ======================================================================
# Preamble
# ======================================================================
cmake_minimum_required(VERSION 3.16)
project(rj_topic_utils)

# ======================================================================
# Find package
# ======================================================================
find_package(rclcpp REQUIRED)

# ======================================================================
# Define Targets
# ======================================================================
add_library(rj_topic_utils INTERFACE)

# ======================================================================
# Dependencies List
# ======================================================================
set(RJ_TOPIC_UTILS_DEPS_INCLUDE_DIRS include)

# ======================================================================
# Include and Linking
# ======================================================================
target_include_directories(rj_topic_utils INTERFACE ${RJ_TOPIC_UTILS_DEPS_INCLUDE_DIRS})

ament_target_dependencies(rj_topic_utils INTERFACE rclcpp)

# ======================================================================
# Packaging
# ======================================================================
install(
  TARGETS rj_topic_utils
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
