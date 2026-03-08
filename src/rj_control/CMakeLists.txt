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
find_package(spdlog REQUIRED)
find_package(fmt REQUIRED)
find_package(rj_control_msgs REQUIRED)
find_package(rj_control_extensions REQUIRED)
find_package(rj_utils REQUIRED)
find_package(rj_constants REQUIRED)
find_package(rj_msgs REQUIRED)
find_package(rj_param_utils REQUIRED)
find_package(rj_topic_utils REQUIRED)
find_package(rj_common REQUIRED)
find_package(rj_geometry REQUIRED)
find_package(rj_convert REQUIRED)
find_package(osqp REQUIRED)
find_package(OsqpEigen REQUIRED)

set(RJ_CONTROL_DEPS
  rclcpp
  std_msgs
  spdlog
  Flann
  fmt
  osqp
  OsqpEigen
  rj_control_msgs
  rj_control_extensions
  rj_utils
  rj_constants
  rj_msgs
  rj_param_utils
  rj_topic_utils
  rj_common
  rj_geometry
  rj_convert
)

set(RJ_CONTROL_LIBS
  spdlog
  fmt
  osqp::osqpstatic
  OsqpEigen::OsqpEigen
  ${FLANN_LIBRARIES}
)

set(RJ_CONTROL_SRC
  # Controllers
  src/controllers/controller.cpp
  src/controllers/position_controller.cpp
  src/controllers/pose_controller.cpp
  src/controllers/shoot_controller.cpp
  src/controllers/collect_controller.cpp
  src/controllers/pass_controller.cpp
  src/controllers/dribble_controller.cpp
  src/controllers/clear_controller.cpp
  src/controllers/mark_robot_controller.cpp
  src/controllers/rotate_controller.cpp

  # Skills
  src/skills/go_to_pose.cpp
  src/skills/go_to_position.cpp
  src/skills/rotate.cpp

  # Barrier Certificates
  src/barriers/linear_cbf.cpp
  src/barriers/cubic_cbf.cpp

  # Utilities
  src/utilities/translational_pid.cpp
  src/utilities/rotational_pid.cpp
)

# RJ Control Library
add_library(${PROJECT_NAME} SHARED
  ${RJ_CONTROL_SRC}
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME}
  PUBLIC
  ${RJ_CONTROL_LIBS}
)

ament_target_dependencies(${PROJECT_NAME}
  PUBLIC
  ${RJ_CONTROL_DEPS}
)

# Control Node
add_executable(control_node
  ${RJ_CONTROL_SRC}
  src/control_node.cpp
)

target_include_directories(control_node
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(control_node
  PUBLIC
  ${RJ_CONTROL_LIBS}
)

ament_target_dependencies(control_node
  PUBLIC
  ${RJ_CONTROL_DEPS}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(
  TARGETS control_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(
  ${RJ_CONTROL_DEPS}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  set(RJ_CONTROL_TESTS
    linear_cbf_test
    cubic_cbf_test
  )

  foreach(TEST IN LISTS RJ_CONTROL_TESTS)
    ament_add_gtest(${TEST} tests/${TEST}.cpp)
    target_include_directories(${TEST} PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
    target_link_libraries(${TEST} ${PROJECT_NAME} ${RJ_CONTROL_LIBS})
  endforeach()
endif()

ament_export_targets(${PROJECT_NAME})
ament_package()
