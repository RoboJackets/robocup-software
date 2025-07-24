# how to write/edit this file: https://docs.ros.org/en/humble/How-To-Guides/Ament-CMake-Documentation.html

cmake_minimum_required(VERSION 3.16)
project(rj_config LANGUAGES CXX)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Enable useful warnings for GCC/Clang
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find required dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rj_msgs REQUIRED)
find_package(fmt REQUIRED)
find_package(rj_constants REQUIRED)
find_package(rj_common REQUIRED)
find_package(rj_utils REQUIRED)

# create library
add_library(config_client STATIC
    src/config_client/config_client.cpp
    src/config_client/config_client_node.cpp)

target_compile_options(config_client PRIVATE "-fPIC")

# create executable
add_executable(config_server 
    src/config_server/config_server.cpp
    src/config_server/main.cpp)

# Include directories
target_include_directories(config_client PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_include_directories(config_server PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Link libraries 
target_link_libraries(config_client PUBLIC
  rj_constants::rj_constants
  fmt::fmt
)

target_link_libraries(config_server PRIVATE
  rj_constants::rj_constants
  fmt::fmt
)

# ament dependencies
ament_target_dependencies(config_client PUBLIC
  rclcpp
  rj_msgs
  rj_common
  rj_utils
)

ament_target_dependencies(config_server PUBLIC
  rclcpp
  rj_msgs
  rj_common
  rj_utils
)

# Installation rules
install(
  TARGETS config_client config_server
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install headers 
install(DIRECTORY include/
  DESTINATION include
)

# Export targets for downstream packages
ament_export_targets(export_${PROJECT_NAME})

# Export dependencies for downstream packages
ament_export_dependencies(
  rclcpp
  rj_msgs
  rj_utils
  rj_common
)

ament_package()