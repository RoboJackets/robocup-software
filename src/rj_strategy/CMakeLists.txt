cmake_minimum_required(VERSION 3.8)
project(rj_strategy)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(spdlog REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rj_msgs REQUIRED)
find_package(rj_constants REQUIRED)
find_package(rj_convert REQUIRED)
find_package(rj_common REQUIRED)
find_package(rj_geometry REQUIRED)
find_package(rj_param_utils REQUIRED)
find_package(rj_utils REQUIRED)

set(RJ_STRATEGY_DEPS
  rclcpp
  rclcpp_action
  rclcpp_components
  spdlog
  std_msgs
  rj_msgs
  rj_constants
  rj_convert
  rj_common
  rj_geometry
  rj_param_utils
  rj_utils
)

set(RJ_STRATEGY_SRC
  # Positions
  src/agent/position.cpp
  src/agent/position/defense.cpp
  src/agent/position/free_kicker.cpp
  src/agent/position/goal_kicker.cpp
  src/agent/position/goalie.cpp
  src/agent/position/idle.cpp
  src/agent/position/line.cpp
  src/agent/position/offense.cpp
  src/agent/position/penalty_non_kicker.cpp
  src/agent/position/penalty_player.cpp
  src/agent/position/pivot_test.cpp
  src/agent/position/robot_factory_position.cpp
  src/agent/position/seeker.cpp
  src/agent/position/smartidling.cpp
  src/agent/position/solo_offense.cpp
  src/agent/position/zoner.cpp

  # Communication
  src/agent/communication.cpp

  # Coordinators
  src/coordinator/kicker_picker_client.cpp
  src/coordinator/marking_client.cpp
  src/coordinator/waller_client.cpp
)

# rj_strategy library
add_library(${PROJECT_NAME} SHARED
  ${RJ_STRATEGY_SRC}
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  PUBLIC
  ${RJ_STRATEGY_DEPS}
)

# Straight Line Test Node
add_executable(straight_line_test_node
  src/testing/straight_line_test.cpp
)
target_link_libraries(straight_line_test_node PRIVATE ${PROJECT_NAME})

# Kicker Picker Node
add_executable(kicker_picker_node
  src/coordinator/kicker_picker.cpp
)
target_link_libraries(kicker_picker_node PRIVATE ${PROJECT_NAME})

# Marking Node
add_executable(marking_node
  src/coordinator/marking.cpp
)
target_link_libraries(marking_node PRIVATE ${PROJECT_NAME})

# Waller Node
add_executable(waller_node
  src/coordinator/waller.cpp
)
target_link_libraries(waller_node PRIVATE ${PROJECT_NAME})

# Agent Action Clients
add_executable(agent_action_clients
  src/agent/agent_action_client.cpp
)
target_link_libraries(agent_action_clients PRIVATE ${PROJECT_NAME})

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

install(
  TARGETS straight_line_test_node kicker_picker_node waller_node marking_node agent_action_clients
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(
  ${RJ_STRATEGY_DEPS}
)

ament_export_targets(${PROJECT_NAME})
ament_package()
