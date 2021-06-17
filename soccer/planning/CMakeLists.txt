# Library
add_library(planning_types)
target_sources(planning_types PRIVATE
        planning_params.cpp
        trajectory.cpp)
target_link_libraries(planning_types PUBLIC
        ${SOCCER_COMMON_LIBS}
        ${SOCCER_COMMON_SYSTEM_LIBS}
        soccer_common)
target_include_directories(planning_types SYSTEM PUBLIC ${EIGEN_INCLUDE_DIRECTORIES} ${RRT_INCLUDE_DIR})
ament_target_dependencies(planning_types PUBLIC ${SOCCER_COMMON_AMENT_LIBS})

add_library(planning)
target_sources(planning PRIVATE
        trajectory_utils.cpp
        planner/collect_planner.cpp
        planner/escape_obstacles_path_planner.cpp
        planner/intercept_planner.cpp
        planner/line_kick_planner.cpp
        planner/path_target_planner.cpp
        planner/pivot_path_planner.cpp
        planner/plan_request.cpp
        planner/settle_planner.cpp
        primitives/angle_planning.cpp
        primitives/create_path.cpp
        primitives/path_smoothing.cpp
        primitives/replanner.cpp
        primitives/rrt_util.cpp
        primitives/trapezoidal_motion.cpp
        primitives/velocity_profiling.cpp)
target_link_libraries(planning PUBLIC
        ${SOCCER_COMMON_LIBS}
        ${SOCCER_COMMON_SYSTEM_LIBS}
        RRT
        soccer_common
        planning_types)
target_include_directories(planning SYSTEM PUBLIC ${EIGEN_INCLUDE_DIRECTORIES} ${RRT_INCLUDE_DIR})
ament_target_dependencies(planning PUBLIC ${SOCCER_COMMON_AMENT_LIBS})

# Node
add_executable(planning_node
        planner_node.cpp
        planner_node_main.cpp)
target_link_libraries(planning_node PUBLIC planning
        config_client)

# Test
if(BUILD_TESTING)
    add_executable(planning_test
            tests/angle_planning_test.cpp
            tests/bezier_path_test.cpp
            tests/conversion_tests.cpp
            tests/create_path_test.cpp
            tests/planner_test.cpp
            tests/testing_utils.cpp
            tests/trajectory_test.cpp
            tests/trapezoidal_motion_test.cpp
            tests/velocity_profiling_test.cpp)
    target_link_libraries(planning_test PUBLIC planning ${GTEST_LIBRARIES} ${GTEST_MAIN_LIBRARIES})
    add_test(NAME planning_test COMMAND planning_test)
endif()