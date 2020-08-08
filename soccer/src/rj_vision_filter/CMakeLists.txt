# ======================================================================
# Set Sources
# ======================================================================
set(RJ_VISION_FILTER_SRC
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

# ======================================================================
# Add sources
# ======================================================================
# ---- robocup ----
target_sources(robocup PRIVATE ${RJ_VISION_FILTER_SRC})

# ---- vision_filter ----
target_sources(rj_vision_filter PRIVATE main.cpp)
