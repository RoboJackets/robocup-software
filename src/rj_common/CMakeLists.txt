cmake_minimum_required(VERSION 3.8)
project(rj_common)

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
find_package(Qt5 COMPONENTS Core Network Widgets REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rj_msgs REQUIRED)
find_package(rj_protos REQUIRED)
find_package(rj_convert REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rj_geometry REQUIRED)
find_package(rj_param_utils REQUIRED)
find_package(rj_utils REQUIRED)
find_package(rj_drawing_msgs REQUIRED)

set(RJ_COMMON_DEPS
  rj_msgs
  Eigen3
  rclcpp
  rclcpp_components
  rj_convert
  rj_protos
  rj_geometry
  rj_param_utils
  rj_utils
  rj_drawing_msgs
)

add_library(${PROJECT_NAME} SHARED
  src/debug_drawer.cpp
  src/field_dimensions.cpp
  src/logger.cpp
  src/multicast.cpp
  src/qt_utils.cpp
  src/referee_enums.cpp
  src/robot_intent.cpp
  src/transforms.cpp
  src/world_state.cpp
  src/planning/motion_command.cpp
  src/planning/trajectory.cpp
  src/radio/packet_convert.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  PUBLIC
  Qt5
  ${RJ_COMMON_DEPS}
)

target_link_libraries(${PROJECT_NAME}
  PUBLIC
  Qt5::Core
  Qt5::Network
  Qt5::Widgets
  Eigen3::Eigen
  ${CMAKE_DL_LIBS}
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_export_dependencies(
  Qt5Core
  Qt5Network
  Qt5Widgets
  ${RJ_COMMON_DEPS}
)

install(
  DIRECTORY include/
  DESTINATION include
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

  set(RJ_COMMON_TESTS
    ball_test
    game_state_test
    logger_test
    rj_common_convert_test
    world_state_test 
  )

  foreach(TEST IN LISTS RJ_COMMON_TESTS)
    ament_add_gtest(${TEST} tests/${TEST}.cpp)
    target_include_directories(${TEST} PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
    target_link_libraries(${TEST} ${PROJECT_NAME})
  endforeach()
endif()

ament_export_targets(${PROJECT_NAME})
ament_package()
