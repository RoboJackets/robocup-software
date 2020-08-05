# ======================================================================
# Set Sources
# ======================================================================
set(RJ_VISION_FILTER_TESTING_SRC
    CameraBallTest.cpp
    KalmanBallTest.cpp
    WorldBallTest.cpp
    CameraRobotTest.cpp
    KalmanRobotTest.cpp
    WorldRobotTest.cpp
    BallBounceTest.cpp
    CameraTest.cpp)

# ======================================================================
# Add sources
# ======================================================================
target_sources(test-soccer PRIVATE ${RJ_VISION_FILTER_TESTING_SRC})
