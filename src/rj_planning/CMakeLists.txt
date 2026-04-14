cmake_minimum_required(VERSION 3.8)
project(rj_planning)

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
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(spdlog REQUIRED)
find_package(fmt REQUIRED)
find_package(Flann REQUIRED)
find_package(rj_geometry REQUIRED)
find_package(rj_common REQUIRED)
find_package(rj_constants REQUIRED)
find_package(rj_convert REQUIRED)
find_package(rj_msgs REQUIRED)
find_package(rj_param_utils REQUIRED)
find_package(rj_rrt REQUIRED)

set(RJ_PLANNING_DEPS
  rclcpp
  rclcpp_action
  rclcpp_components
  spdlog
  fmt
  rj_geometry
  rj_common
  rj_constants
  rj_convert
  rj_msgs
  rj_param_utils
  rj_rrt
  Flann
)

set(RJ_PLANNING_LIBS
  spdlog
  fmt
  ${FLANN_LIBRARIES}
)

set(RJ_PLANNING_SRC
  src/global_state.cpp
  src/plan_request.cpp
  src/planner_for_robot.cpp
  src/planner_node.cpp
  src/trajectory_collection.cpp
  src/trajectory_utils.cpp

  # Planners
  src/planners/collect_path_planner.cpp
  src/planners/escape_obstacles_path_planner.cpp
  src/planners/goalie_idle_path_planner.cpp
  src/planners/intercept_path_planner.cpp
  src/planners/line_kick_path_planner.cpp
  src/planners/line_pivot_path_planner.cpp
  src/planners/path_target_path_planner.cpp
  src/planners/pivot_path_planner.cpp
  src/planners/rotate_path_planner.cpp
  src/planners/settle_path_planner.cpp

  # Primitives
  src/primitives/angle_planning.cpp
  src/primitives/create_path.cpp
  src/primitives/path_smoothing.cpp
  src/primitives/replanner.cpp
  src/primitives/rrt_util.cpp
  src/primitives/trapezoidal_motion.cpp
  src/primitives/velocity_profiling.cpp

  # Utilities
  src/utils/testing_utils.cpp
)

# RJ Planning Library
add_library(${PROJECT_NAME} SHARED
  ${RJ_PLANNING_SRC}
)
set_target_properties(${PROJECT_NAME} PROPERTIES UNITY_BUILD ON)

target_include_directories(${PROJECT_NAME}
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME}
  PUBLIC
  ${RJ_PLANNING_LIBS}
)

ament_target_dependencies(${PROJECT_NAME}
  PUBLIC
  ${RJ_PLANNING_DEPS}
)

# Planner Node
add_executable(planner_node
  src/main.cpp
)

target_include_directories(planner_node
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(planner_node
  PRIVATE
  ${PROJECT_NAME}
)

ament_target_dependencies(planner_node
  ${RJ_PLANNING_DEPS}
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

install(
  TARGETS planner_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(
  ${RJ_PLANNING_DEPS}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  set(RJ_PLANNING_TESTS
    angle_planning_test
    bezier_path_test
    conversion_tests
    create_path_test
    planner_test
    trajectory_test
    trapezoidal_motion_test
    velocity_profiling_test
  )

  foreach(TEST IN LISTS RJ_PLANNING_TESTS)
    ament_add_gtest(${TEST} tests/${TEST}.cpp)
    target_include_directories(${TEST} PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include)
    target_link_libraries(${TEST} ${PROJECT_NAME})
  endforeach()
endif()

ament_export_targets(${PROJECT_NAME})
ament_package()
