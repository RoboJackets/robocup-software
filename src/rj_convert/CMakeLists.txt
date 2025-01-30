cmake_minimum_required(VERSION 3.16)
project(rj_convert)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# Declare header-only library
add_library(rj_convert INTERFACE)
target_include_directories(rj_convert INTERFACE include)
ament_target_dependencies(rj_convert INTERFACE rclcpp)

ament_export_targets(rj_convert HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  add_subdirectory(test)
endif()

# Install headers
install(DIRECTORY include/ DESTINATION include/)

# Install the target
install(
  TARGETS rj_convert
  INCLUDES DESTINATION include
)

ament_package()
