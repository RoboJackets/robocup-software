#======================================================================
# Preamble
#======================================================================
cmake_minimum_required(VERSION 3.16)
project(rj_utils LANGUAGES CXX)

#======================================================================
# Find package
#======================================================================
find_package(rclcpp REQUIRED)

#======================================================================
# Define Targets
#======================================================================
add_library(rj_utils INTERFACE)
target_include_directories(rj_utils INTERFACE include)
ament_target_dependencies(rj_utils INTERFACE rclcpp)

#======================================================================
# Packaging
#======================================================================
install(DIRECTORY include/
    DESTINATION include/)
