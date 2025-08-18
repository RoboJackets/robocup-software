cmake_minimum_required(VERSION 3.8)
project(rj_joystick)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(fmt REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(spdlog REQUIRED)
find_package(rj_utils REQUIRED)
find_package(rj_common REQUIRED)
find_package(rj_constants REQUIRED)
find_package(rj_msgs REQUIRED)
find_package(rj_param_utils REQUIRED)

# Find sdl2
include(FindPkgConfig)
pkg_search_module(SDL2 REQUIRED sdl2)

add_executable(manual_control_node
  src/manual_control_node.cpp
  src/sdl_manual_control.cpp
)

target_link_libraries(manual_control_node
  PUBLIC
  spdlog
  fmt
  ${SDL2_LIBRARIES}
)

target_include_directories(manual_control_node
  PUBLIC
  ${SDL2_INCLUDE_DIRS}
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(manual_control_node
  PUBLIC
  fmt
  rclcpp
  spdlog
  rj_utils
  rj_common
  rj_constants
  rj_msgs
  rj_param_utils
  rclcpp_components
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS manual_control_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
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
