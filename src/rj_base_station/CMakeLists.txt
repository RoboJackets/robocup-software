cmake_minimum_required(VERSION 3.8)
project(rj_base_station)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-comment)
endif()

add_subdirectory(external/RF24)
add_subdirectory(external/pigpio)

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rj_common REQUIRED)
find_package(rj_msgs REQUIRED)
find_package(rj_constants REQUIRED)

set(RJ_BASE_STATION_DEPS
  rclcpp
  rclcpp_components
  rj_common
  rj_msgs
  rj_constants
)

add_executable(radio_node
  src/rf_radio_node.cpp
)

add_executable(alive_robots_node
  src/alive_robots_node.cpp
)

# Include RF24 Radio Library with the radio node
target_link_libraries(radio_node
  PUBLIC
  rf24
  pigpio
  pthread
  ${CMAKE_DL_LIBS}
)

target_include_directories(radio_node
  PUBLIC
  external/RF24
  external/pigpio
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_include_directories(alive_robots_node
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(radio_node
  PUBLIC
  ${RJ_BASE_STATION_DEPS}
)

ament_target_dependencies(alive_robots_node
  PUBLIC
  ${RJ_BASE_STATION_DEPS}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS radio_node alive_robots_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
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
