cmake_minimum_required(VERSION 3.16)
project(rj_convert)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# Declare header-only library
add_library(rj_convert INTERFACE)
target_include_directories(rj_convert INTERFACE include)
ament_target_dependencies(rj_convert INTERFACE rclcpp)

ament_export_dependencies(rclcpp)

# Install headers
install(DIRECTORY include/ DESTINATION include/)

# Install the target (without EXPORT)
install(
  TARGETS rj_convert
  INCLUDES DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_rj_convert test/ros_convert_test.cpp)

  # Ensure tests find headers
  target_include_directories(test_rj_convert PRIVATE include)

  # Link test with rj_convert and rclcpp
  target_link_libraries(test_rj_convert rj_convert)
  ament_target_dependencies(test_rj_convert rclcpp)

endif()

ament_package()
