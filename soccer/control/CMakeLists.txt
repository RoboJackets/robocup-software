# Library
add_library(control)
target_sources(control PRIVATE
        manipulator_control.cpp
        motion_control.cpp
        trapezoidal_motion.cpp)
target_link_libraries(control PUBLIC
        ${SOCCER_COMMON_LIBS}
        ${SOCCER_COMMON_SYSTEM_LIBS}
        planning_types
        soccer_common
        rc-fshare)
target_include_directories(control SYSTEM PUBLIC ${EIGEN_INCLUDE_DIRECTORIES})
ament_target_dependencies(control PUBLIC ${SOCCER_COMMON_AMENT_LIBS})

# Node
add_executable(control_node
        motion_control_node.cpp
        control_node_main.cpp)
target_link_libraries(control_node PUBLIC
        control
        config_client)

# Test
if(BUILD_TESTING)
    add_executable(control_test
            motion_control_test.cpp
            trapezoidal_motion_test.cpp)
    target_link_libraries(control_test PUBLIC control ${GTEST_LIBRARIES} ${GTEST_MAIN_LIBRARIES})
    add_test(NAME control_test COMMAND control_test)
endif()
