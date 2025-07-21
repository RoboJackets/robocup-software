# ======================================================================
# Preamble
# ======================================================================
cmake_minimum_required(VERSION 3.16)
project(rj_config)

# ======================================================================
# Find package
# ======================================================================
find_package(rclcpp REQUIRED)
find_package(rj_msgs REQUIRED)
find_package(fmt REQUIRED)

# ======================================================================
# Define Targets
# ======================================================================
add_library(config_client STATIC)
target_compile_options(config_client PRIVATE "-fPIC")

add_executable(config_server)

add_subdirectory(src)

# ======================================================================
# Dependencies List
# ======================================================================
set(RJ_CONFIG_DEPS_INCLUDE_DIRS include)

set(RJ_CONFIG_DEPS_LIBRARIES rj_constants rj_common rj_utils fmt)

# ======================================================================
# Include and Linking
# ======================================================================
target_include_directories(config_client PUBLIC ${RJ_CONFIG_DEPS_INCLUDE_DIRS})
target_include_directories(config_server PRIVATE ${RJ_CONFIG_DEPS_INCLUDE_DIRS})

target_link_libraries(config_client PUBLIC ${RJ_CONFIG_DEPS_LIBRARIES})
target_link_libraries(config_server PRIVATE ${RJ_CONFIG_DEPS_LIBRARIES})

ament_target_dependencies(
  config_client
  PUBLIC
  rclcpp
  rj_msgs)
ament_target_dependencies(
  config_server
  PUBLIC
  rclcpp
  rj_msgs)

# ======================================================================
# Packaging
# ======================================================================
install(
  TARGETS config_client
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS config_server DESTINATION lib/${CMAKE_PROJECT_NAME})
