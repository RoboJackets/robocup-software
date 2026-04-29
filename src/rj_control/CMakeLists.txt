cmake_minimum_required(VERSION 3.8)
project(rj_control)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Flann REQUIRED)
find_package(rj_utils REQUIRED)
find_package(rj_constants REQUIRED)
find_package(rj_msgs REQUIRED)
find_package(rj_param_utils REQUIRED)
find_package(rj_topic_utils REQUIRED)
find_package(rj_common REQUIRED)
find_package(rj_geometry REQUIRED)
find_package(rj_planning REQUIRED)
find_package(rj_convert REQUIRED)

set(RJ_CONTROL_DEPS
  std_msgs
  Flann
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

add_executable(motion_control_node
  src/motion_control_node.cpp
  src/motion_control.cpp
  src/pid.cpp
)

add_library(${PROJECT_NAME} SHARED
  src/motion_control_node.cpp
  src/motion_control.cpp
  src/pid.cpp
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

target_link_libraries(motion_control_node
  PUBLIC
  ${FLANN_LIBRARIES}
)

target_link_libraries(${PROJECT_NAME}
  PUBLIC
  ${FLANN_LIBRARIES}
)

ament_target_dependencies(motion_control_node
  PUBLIC
  rclcpp
  ${RJ_CONTROL_DEPS}
)

ament_target_dependencies(${PROJECT_NAME}
  PUBLIC
  rclcpp
  ${RJ_CONTROL_DEPS}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS motion_control_node
  DESTINATION lib/${PROJECT_NAME}
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
  ${RJ_CONTROL_DEPS}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  
  # Motion Control Tests
  ament_add_gtest(motion_control_test tests/motion_control_test.cpp)
  target_include_directories(motion_control_test PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
  target_link_libraries(motion_control_test ${PROJECT_NAME})
endif()

ament_export_targets(${PROJECT_NAME})
ament_package()
