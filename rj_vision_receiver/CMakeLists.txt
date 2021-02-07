# ======================================================================
# Preamble
# ======================================================================
cmake_minimum_required(VERSION 3.16)
project(rj_vision_receiver)

# ======================================================================
# Find package
# ======================================================================
find_package(rclcpp REQUIRED)
find_package(Boost REQUIRED)
find_package(rj_msgs REQUIRED)

# ======================================================================
# Define Targets
# ======================================================================
add_executable(vision_receiver)

add_library(rj_vision_receiver STATIC)

add_subdirectory(src)

# ======================================================================
# Dependencies List
# ======================================================================
set(RJ_VISION_RECEIVER_DEPS_INCLUDE_DIRS include)

set(RJ_VISION_RECEIVER_DEPS_SYSTEM_LIBRARIES Boost::asio)

set(RJ_VISION_RECEIVER_DEPS_LIBRARIES
    config_client
    rj_common
    rj_param_utils
    rj_utils)

set(RJ_VISION_RECEIVER_SRCS
    src/vision_receiver.cpp)

# ======================================================================
# Include and Linking
# ======================================================================
target_include_directories(vision_receiver PRIVATE ${RJ_VISION_RECEIVER_DEPS_INCLUDE_DIRS})

target_link_libraries(vision_receiver PRIVATE ${RJ_VISION_RECEIVER_DEPS_LIBRARIES})

ament_target_dependencies(
  vision_receiver
  PUBLIC
  rclcpp
  rj_msgs)

target_include_directories(rj_vision_receiver PUBLIC ${RJ_VISION_RECEIVER_DEPS_INCLUDE_DIRS})

target_link_libraries(rj_vision_receiver PUBLIC ${RJ_VISION_RECEIVER_DEPS_LIBRARIES})

target_sources(rj_vision_receiver PRIVATE ${RJ_VISION_RECEIVER_SRCS})

ament_target_dependencies(
  rj_vision_receiver
  PUBLIC
  rclcpp
  rj_msgs)

# ======================================================================
# Packaging
# ======================================================================
install(TARGETS vision_receiver DESTINATION lib/${CMAKE_PROJECT_NAME})

install(
  TARGETS rj_vision_receiver
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
