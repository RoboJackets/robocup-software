find_package(ament_cmake REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(rj_convert REQUIRED)
find_package(rclcpp REQUIRED)

# Define Google Test
ament_add_gtest(test_rj_convert ros_convert_test.cpp)
target_link_libraries(test_rj_convert rj_convert)
ament_target_dependencies(test_rj_convert rclcpp)

ament_add_test(
  NAME test_rj_convert
  COMMAND test_rj_convert
)
