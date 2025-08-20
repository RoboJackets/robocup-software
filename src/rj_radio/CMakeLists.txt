cmake_minimum_required(VERSION 3.8)
project(rj_radio)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(spdlog REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(Boost REQUIRED)
find_package(Flann REQUIRED)
find_package(rj_common REQUIRED)
find_package(rj_msgs REQUIRED)
find_package(rj_protos REQUIRED)
find_package(rj_utils REQUIRED)
find_package(rj_param_utils REQUIRED)
find_package(rj_strategy REQUIRED)
find_package(rj_geometry REQUIRED)
find_package(rj_planning REQUIRED)
find_package(rj_constants REQUIRED)

add_executable(network_radio_node
  src/network_radio.cpp
  src/radio.cpp
)

add_executable(sim_radio_node
  src/sim_radio.cpp
  src/radio.cpp
)

target_link_libraries(network_radio_node
  PUBLIC
  spdlog
  Boost::boost
  ${FLANN_LIBRARIES}
)

target_link_libraries(sim_radio_node
  PUBLIC
  spdlog
  Boost::boost
  ${FLANN_LIBRARIES}
)

target_include_directories(network_radio_node
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_include_directories(sim_radio_node
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(network_radio_node
  PUBLIC
  spdlog
  rclcpp
  rclcpp_components
  Boost
  Flann
  rj_common
  rj_msgs
  rj_protos
  rj_utils
  rj_param_utils
  rj_strategy
  rj_geometry
  rj_planning
  rj_constants
)

ament_target_dependencies(sim_radio_node
  PUBLIC
  spdlog
  rclcpp
  rclcpp_components
  Boost
  Flann
  rj_common
  rj_msgs
  rj_protos
  rj_utils
  rj_param_utils
  rj_strategy
  rj_geometry
  rj_planning
  rj_constants
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS network_radio_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  TARGETS sim_radio_node
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
