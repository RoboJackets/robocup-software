cmake_minimum_required(VERSION 3.8)
project(rj_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rj_utils REQUIRED)
find_package(rj_constants REQUIRED)
find_package(rj_msgs REQUIRED)
find_package(rj_param_utils REQUIRED)
find_package(rj_topic_utils REQUIRED)
find_package(rj_common REQUIRED)
find_package(rj_geometry REQUIRED)
find_package(rj_planning REQUIRED)
find_package(rj_convert REQUIRED)

add_executable(motion_control_node
  src/motion_control_node.cpp
  src/motion_control.cpp
  src/pid.cpp
  src/trapezoidal_motion.cpp
)

add_library(${PROJECT_NAME} SHARED
  src/motion_control_node.cpp
  src/motion_control.cpp
  src/pid.cpp
  src/trapezoidal_motion.cpp
)

target_include_directories(motion_control_node
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(motion_control_node
  PUBLIC
  rclcpp
  std_msgs
  rj_utils
  rj_constants
  rj_msgs
  rj_param_utils
  rj_topic_utils
  rj_common
  rj_geometry
  rj_planning
  rj_convert
)

ament_target_dependencies(${PROJECT_NAME}
  PUBLIC
  rclcpp
  std_msgs
  rj_utils
  rj_constants
  rj_msgs
  rj_param_utils
  rj_topic_utils
  rj_common
  rj_geometry
  rj_planning
  rj_convert
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS motion_control_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_export_dependencies(
  std_msgs
  rj_utils
  rj_constants
  rj_msgs
  rj_param_utils
  rj_topic_utils
  rj_common
  rj_geometry
  rj_planning
  rj_convert
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)

  # Motion Control Tests
  ament_add_gtest(motion_control_test tests/motion_control_test.cpp)
  target_include_directories(motion_control_test PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
  target_link_libraries(motion_control_test ${PROJECT_NAME})

  # Trapezoidal Motion Tests
  ament_add_gtest(trapezoidal_motion_test tests/trapezoidal_motion_test.cpp)
  target_include_directories(trapezoidal_motion_test PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
  target_link_libraries(trapezoidal_motion_test ${PROJECT_NAME})
endif()

ament_export_targets(${PROJECT_NAME})
ament_package()
