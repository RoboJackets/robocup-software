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

set(RJ_CONFIG_DEPS_LIBRARIES
    rj_constants
    rj_utils)

#======================================================================
# Include and Linking
#======================================================================
target_include_directories(config_client
    PUBLIC
    ${RJ_CONFIG_DEPS_INCLUDE_DIRS})
target_include_directories(config_server
    PRIVATE
    ${RJ_CONFIG_DEPS_INCLUDE_DIRS})

target_link_libraries(config_client
    PUBLIC
    ${RJ_CONFIG_DEPS_LIBRARIES})
target_link_libraries(config_server
    PRIVATE
    ${RJ_CONFIG_DEPS_LIBRARIES})

ament_target_dependencies(config_client PUBLIC rclcpp)
ament_target_dependencies(config_server PUBLIC rclcpp)

include(ROSIDLUtils)
rosidl_target_link_interfaces(config_client PUBLIC rj_msgs)
rosidl_target_link_interfaces(config_server PRIVATE rj_msgs)

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
