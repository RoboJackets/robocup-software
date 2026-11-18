cmake_minimum_required(VERSION 3.8)
project(rj_game_common)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(spdlog REQUIRED)
find_package(rj_geometry REQUIRED)
find_package(rj_referee_msgs REQUIRED)
find_package(rj_vision_msgs REQUIRED)

set(RJ_GAME_COMMON_DEPS
  rclcpp
  spdlog
  rj_geometry
  rj_referee_msgs
  rj_vision_msgs
)

add_library(${PROJECT_NAME} INTERFACE)

target_include_directories(${PROJECT_NAME}
  INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  INTERFACE
  ${RJ_GAME_COMMON_DEPS}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  INCLUDES DESTINATION include
)

ament_export_dependencies(${RJ_GAME_COMMON_DEPS})
ament_export_targets(${PROJECT_NAME})
ament_package()
