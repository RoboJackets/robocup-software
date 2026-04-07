cmake_minimum_required(VERSION 3.8)
project(rj_vision_filter)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rj_geometry REQUIRED)
find_package(rj_common REQUIRED)
find_package(rj_msgs REQUIRED)
find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(rj_param_utils REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rj_utils REQUIRED)
find_package(spdlog REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rj_config_client REQUIRED)
find_package(rj_topic_utils REQUIRED)
find_package(rj_constants REQUIRED)

set(RJ_VISION_FILTER_DEPS
  rj_geometry
  rj_common
  rj_msgs
  Boost
  Eigen3
  eigen3_cmake_module
  rj_param_utils
  rclcpp
  rj_utils
  spdlog
  rclcpp_components
  rj_config_client
  rj_topic_utils
  rj_constants
)

add_executable(${PROJECT_NAME}_node
  src/main.cpp
)

add_library(${PROJECT_NAME}
  src/ball/ball_bounce.cpp
  src/ball/camera_ball.cpp
  src/ball/kalman_ball.cpp
  src/ball/world_ball.cpp
  src/camera/camera.cpp
  src/camera/world.cpp
  src/filter/kalman_filter.cpp
  src/filter/kalman_filter_2d.cpp
  src/filter/kalman_filter3_d.cpp
  src/kick/detector/fast_kick_detector.cpp
  src/kick/detector/slow_kick_detector.cpp
  src/kick/estimator/chip_kick_estimator.cpp
  src/kick/estimator/flat_kick_estimator.cpp
  src/kick/kick_event.cpp
  src/robot/camera_robot.cpp
  src/robot/kalman_robot.cpp
  src/robot/world_robot.cpp
  src/vision_filter.cpp
)

target_link_libraries(${PROJECT_NAME}_node
  PRIVATE
  ${PROJECT_NAME}
)

target_link_libraries(${PROJECT_NAME}
  PUBLIC
  Boost::boost
  Eigen3::Eigen
  spdlog
)

target_include_directories(${PROJECT_NAME}_node
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}_node
  PUBLIC
  ${RJ_VISION_FILTER_DEPS}
)

ament_target_dependencies(${PROJECT_NAME}
  PUBLIC
  ${RJ_VISION_FILTER_DEPS}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}_node
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

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  set(RJ_VISION_FILTER_TESTS
    ball_bounce_test
    camera_ball_test
    camera_robot_test
    camera_test
    kalman_ball_test
    kalman_robot_test
    world_ball_test
    world_robot_test
  )

  foreach(TEST IN LISTS RJ_VISION_FILTER_TESTS)
    ament_add_gtest(${TEST} tests/${TEST}.cpp)
    target_include_directories(${TEST} PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include)
    target_link_libraries(${TEST} ${PROJECT_NAME})
  endforeach()
endif()

ament_export_targets(${PROJECT_NAME})
ament_package()
