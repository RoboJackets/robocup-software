cmake_minimum_required(VERSION 3.8)
project(rj_testing)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

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
find_package(spdlog REQUIRED)
find_package(fmt REQUIRED)
find_package(rj_control_msgs REQUIRED)
find_package(rj_control_extensions REQUIRED)
find_package(rj_common REQUIRED)
find_package(rj_convert REQUIRED)
find_package(rj_geometry REQUIRED)
find_package(rj_utils REQUIRED)

set(RJ_TESTING_DEPS
  rclcpp
  std_msgs
  spdlog
  fmt
  rj_control_msgs
  rj_control_extensions
  rj_common
  rj_convert
  rj_geometry
  rj_utils
)

set(RJ_TESTING_LIBS
  spdlog
  fmt
)

# Line Test Node
add_executable(line_test_node
  src/line_test_node.cpp
)

target_include_directories(line_test_node
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(line_test_node
  PUBLIC
  ${RJ_TESTING_LIBS}
)

ament_target_dependencies(line_test_node
  PUBLIC
  ${RJ_TESTING_DEPS}
)

# Rotate Test Node
add_executable(rotate_test_node
  src/rotate_test_node.cpp
)

target_include_directories(rotate_test_node
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(rotate_test_node
  PUBLIC
  ${RJ_TESTING_LIBS}
)

ament_target_dependencies(rotate_test_node
  PUBLIC
  ${RJ_TESTING_DEPS}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS line_test_node rotate_test_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(
  ${RJ_TESTING_DEPS}
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
endif()

ament_package()
