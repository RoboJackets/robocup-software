# ======================================================================
# Set Sources
# ======================================================================
set(RJ_VISION_FILTER_TESTING_SRC
    camera_ball_test.cpp
    kalman_ball_test.cpp
    world_ball_test.cpp
    camera_robot_test.cpp
    kalman_robot_test.cpp
    world_robot_test.cpp
    ball_bounce_test.cpp
    camera_test.cpp)

# ======================================================================
# Add sources
# ======================================================================
target_sources(test-soccer PRIVATE ${RJ_VISION_FILTER_TESTING_SRC})
