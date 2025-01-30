cmake_minimum_required(VERSION 3.16)
project(rj_param_utils)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)  # Ensure rclcpp is found

# Define shared library
add_library(rj_param_utils SHARED)

# Add source files
target_sources(rj_param_utils PRIVATE
    src/param.cpp
    src/ros2_param_provider.cpp
    src/ros2_local_param_provider.cpp
    src/ros2_global_param_provider.cpp
)

# Include directories
target_include_directories(rj_param_utils PUBLIC include)

# Link dependencies properly
ament_target_dependencies(rj_param_utils PUBLIC rclcpp)

# Install headers
install(DIRECTORY include/ DESTINATION include/)

# Install the library
install(
  TARGETS rj_param_utils
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Testing
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_rj_param_utils
    test/declare_test.cpp
    test/param_test.cpp
    test/global_param_provider_test.cpp
  )
  target_include_directories(test_rj_param_utils PUBLIC include)
  target_link_libraries(test_rj_param_utils rj_param_utils)
endif()

ament_package()
