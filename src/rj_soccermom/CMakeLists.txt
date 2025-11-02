cmake_minimum_required(VERSION 3.16)
project(rj_soccermom)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rj_msgs REQUIRED)
find_package(rj_constants REQUIRED)
find_package(rj_utils REQUIRED)
find_package(fmt REQUIRED)
find_package(Qt5 COMPONENTS Core Widgets REQUIRED)

add_library(soccer_mom_node_lib src/soccer_mom.cpp)
ament_target_dependencies(soccer_mom_node_lib
  PUBLIC
  rclcpp
  std_msgs
  rj_msgs
  rj_constants
  rj_utils
  Qt5
)

target_include_directories(soccer_mom_node_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_executable(soccer_mom_node src/main.cpp)
target_link_libraries(soccer_mom_node PRIVATE soccer_mom_node_lib)
target_link_libraries(soccer_mom_node_lib
  PUBLIC
  fmt
)
ament_target_dependencies(soccer_mom_node PUBLIC rclcpp)

install(TARGETS soccer_mom_node_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS soccer_mom_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

ament_package()