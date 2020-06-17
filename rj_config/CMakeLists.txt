#======================================================================
# Preamble
#======================================================================
cmake_minimum_required(VERSION 3.16)
project(rj_config)

#======================================================================
# Find package
#======================================================================
find_package(rclcpp REQUIRED)

#======================================================================
# Define Targets
#======================================================================
add_library(config_client STATIC)

add_executable(config_server)

add_subdirectory(src)

#======================================================================
# Dependencies List
#======================================================================
set(RJ_CONFIG_DEPS_INCLUDE_DIRS
    include)

#======================================================================
# Include and Linking
#======================================================================
target_include_directories(config_client
    PUBLIC
    ${RJ_CONFIG_DEPS_INCLUDE_DIRS})
target_include_directories(config_server
    PRIVATE
    ${RJ_CONFIG_DEPS_INCLUDE_DIRS})

ament_target_dependencies(config_client rclcpp)
ament_target_dependencies(config_server rclcpp)

include(ROSIDLUtils)
rosidl_target_link_interfaces(config_client rj_msgs)
rosidl_target_link_interfaces(config_server rj_msgs)

#======================================================================
# Packaging
#======================================================================
install(TARGETS
    config_client
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin)

install(TARGETS config_server
    DESTINATION lib/${CMAKE_PROJECT_NAME})
