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

set(RJ_RADIO_DEPS
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
  ${RJ_RADIO_DEPS}
)

ament_target_dependencies(sim_radio_node
  PUBLIC
  ${RJ_RADIO_DEPS}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS network_radio_node sim_radio_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
