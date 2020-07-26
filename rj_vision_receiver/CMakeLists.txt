#======================================================================
# Preamble
#======================================================================
cmake_minimum_required(VERSION 3.16)
project(rj_vision_receiver)

#======================================================================
# Find package
#======================================================================
find_package(rclcpp REQUIRED)
find_package(Boost REQUIRED)
find_package(rj_msgs REQUIRED)

#======================================================================
# Define Targets
#======================================================================
add_executable(vision_receiver)

add_subdirectory(src)

#======================================================================
# Dependencies List
#======================================================================
set(RJ_VISION_RECEIVER_DEPS_INCLUDE_DIRS
    include)

set(RJ_VISION_RECEIVER_DEPS_SYSTEM_LIBRARIES
    Boost::asio)

set(RJ_VISION_RECEIVER_DEPS_LIBRARIES
    config_client
    rj_common
    rj_param_utils
    rj_utils)

#======================================================================
# Include and Linking
#======================================================================
target_include_directories(vision_receiver
    PRIVATE
    ${RJ_VISION_RECEIVER_DEPS_INCLUDE_DIRS})

target_link_libraries(vision_receiver
    PRIVATE
    ${RJ_VISION_RECEIVER_DEPS_LIBRARIES})

ament_target_dependencies(vision_receiver PUBLIC rclcpp rj_msgs)

#======================================================================
# Packaging
#======================================================================
install(TARGETS vision_receiver
    DESTINATION lib/${CMAKE_PROJECT_NAME})
