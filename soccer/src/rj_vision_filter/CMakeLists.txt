#======================================================================
# Set Sources
#======================================================================
set(RJ_VISION_FILTER_SRC
    ball/BallBounce.cpp
    ball/CameraBall.cpp
    ball/KalmanBall.cpp
    ball/WorldBall.cpp
    camera/Camera.cpp
    camera/World.cpp
    filter/KalmanFilter.cpp
    filter/KalmanFilter2D.cpp
    filter/KalmanFilter3D.cpp
    kick/detector/FastKickDetector.cpp
    kick/detector/SlowKickDetector.cpp
    kick/estimator/ChipKickEstimator.cpp
    kick/estimator/FlatKickEstimator.cpp
    kick/KickEvent.cpp
    robot/CameraRobot.cpp
    robot/KalmanRobot.cpp
    robot/WorldRobot.cpp
    util/VisionFilterConfig.cpp
    VisionFilter.cpp)

#======================================================================
# Add sources
#======================================================================
# ---- robocup ----
target_sources(
    robocup
    PRIVATE
    ${RJ_VISION_FILTER_SRC})
