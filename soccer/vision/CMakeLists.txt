# Library
add_library(vision_filter)
target_sources(vision_filter PRIVATE
        ball/ball_bounce.cpp
        ball/camera_ball.cpp
        ball/kalman_ball.cpp
        ball/world_ball.cpp
        camera/camera.cpp
        camera/world.cpp
        filter/kalman_filter.cpp
        filter/kalman_filter_2d.cpp
        filter/kalman_filter3_d.cpp
        kick/detector/fast_kick_detector.cpp
        kick/detector/slow_kick_detector.cpp
        kick/estimator/chip_kick_estimator.cpp
        kick/estimator/flat_kick_estimator.cpp
        kick/kick_event.cpp
        robot/camera_robot.cpp
        robot/kalman_robot.cpp
        robot/world_robot.cpp
        params.cpp
        vision_filter.cpp)
target_link_libraries(vision_filter PUBLIC
        ${SOCCER_COMMON_LIBS}
        ${SOCCER_COMMON_SYSTEM_LIBS}
        soccer_common)
target_include_directories(vision_filter SYSTEM PUBLIC)
ament_target_dependencies(vision_filter PUBLIC ${SOCCER_COMMON_AMENT_LIBS})

# Node
add_executable(vision_filter_node
        main.cpp)
target_link_libraries(vision_filter_node PUBLIC
        vision_filter)

# Test
if(BUILD_TESTING)
    add_executable(vision_filter_test
            test/camera_ball_test.cpp
            test/kalman_ball_test.cpp
            test/world_ball_test.cpp
            test/camera_robot_test.cpp
            test/kalman_robot_test.cpp
            test/world_robot_test.cpp
            test/ball_bounce_test.cpp
            test/camera_test.cpp)
    target_link_libraries(vision_filter_test PUBLIC vision_filter ${GTEST_LIBRARIES} ${GTEST_MAIN_LIBRARIES})
    add_test(NAME vision_filter_test COMMAND vision_filter_test)
endif()
