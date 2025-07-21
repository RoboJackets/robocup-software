# how to write/edit this file: https://docs.ros.org/en/humble/How-To-Guides/Ament-CMake-Documentation.html

cmake_minimum_required(VERSION 3.16)
project(rj_convert LANGUAGES CXX)

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

# create library
add_library(${PROJECT_NAME} INTERFACE)

# add dependencies
ament_target_dependencies(${PROJECT_NAME} INTERFACE rclcpp)

# required for ament
target_include_directories(${PROJECT_NAME} INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(
  DIRECTORY include/
  DESTINATION include/
)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# Export targets and dependencies for other packages
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(rclcpp)

# Enable tests if requested
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  add_subdirectory(testing)
endif()

# Register this package with ament
ament_package()
