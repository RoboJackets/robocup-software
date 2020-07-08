#======================================================================
# Set Sources
#======================================================================
set(SOCCER_UIS
    ui/qt/ConfigFileTab.ui
    ui/qt/LogViewer.ui
    ui/qt/MainWindow.ui
    ui/qt/RefereeTab.ui
    ui/qt/RobotStatusWidget.ui
    )

set(SOCCER_RSRC
    ui/qt/main_icons.qrc
    )

set(ROBOCUP_LIB_SRC
    BatteryProfile.cpp
    Configuration.cpp
    gameplay/GameplayModule.cpp
    gameplay/robocup-py.cpp
    joystick/SDLJoystickNode.cpp
    joystick/SDLGamepad.cpp
    joystick/ManualControlNode.cpp
    KickEvaluator.cpp
    Logger.cpp
    motion/MotionControl.cpp
    motion/MotionControlNode.cpp
    motion/TrapezoidalMotion.cpp
    Referee.cpp
    optimization/GradientAscent1D.cpp
    optimization/ParallelGradientAscent1D.cpp
    optimization/NelderMead2D.cpp
    optimization/PythonFunctionWrapper.cpp
    planning/primitives/AnglePlanning.cpp
    planning/primitives/CreatePath.cpp
    planning/primitives/PathSmoothing.cpp
    planning/primitives/Replanner.cpp
    planning/primitives/RRTUtil.cpp
    planning/primitives/TrapezoidalMotion.cpp
    planning/primitives/VelocityProfiling.cpp
    planning/planner/CollectPlanner.cpp
    planning/planner/EscapeObstaclesPathPlanner.cpp
    planning/planner/InterceptPlanner.cpp
    planning/planner/LineKickPlanner.cpp
    planning/planner/PathTargetPlanner.cpp
    planning/planner/PivotPathPlanner.cpp
    planning/planner/PlanRequest.cpp
    planning/planner/SettlePlanner.cpp
    planning/MotionConstraints.cpp
    planning/PlannerNode.cpp
    planning/RotationConstraints.cpp
    planning/Trajectory.cpp
    planning/TrajectoryUtils.cpp
    Processor.cpp
    radio/NetworkRadio.cpp
    radio/PacketConvert.cpp
    radio/RadioNode.cpp
    radio/SimRadio.cpp
    Robot.cpp
    RobotConfig.cpp
    SystemState.cpp
    DebugDrawer.cpp
    ui/BatteryWidget.cpp
    ui/FieldView.cpp
    ui/MainWindow.cpp
    ui/ProtobufTree.cpp
    ui/RefereeTab.cpp
    ui/RobotStatusWidget.cpp
    ui/RobotWidget.cpp
    ui/SimFieldView.cpp
    ui/StripChart.cpp
    ui/StyleSheetManager.cpp
    GrSimCommunicator.cpp
    vision/ball/BallBounce.cpp
    vision/ball/CameraBall.cpp
    vision/ball/KalmanBall.cpp
    vision/ball/WorldBall.cpp
    vision/camera/Camera.cpp
    vision/camera/World.cpp
    vision/filter/KalmanFilter.cpp
    vision/filter/KalmanFilter2D.cpp
    vision/filter/KalmanFilter3D.cpp
    vision/kick/detector/FastKickDetector.cpp
    vision/kick/detector/SlowKickDetector.cpp
    vision/kick/estimator/ChipKickEstimator.cpp
    vision/kick/estimator/FlatKickEstimator.cpp
    vision/kick/KickEvent.cpp
    vision/robot/CameraRobot.cpp
    vision/robot/KalmanRobot.cpp
    vision/robot/WorldRobot.cpp
    vision/util/VisionFilterConfig.cpp
    vision/VisionFilter.cpp
    WorldState.cpp
    ros2_temp/soccer_config_client.cpp
    ros2_temp/detection_frame_sub.cpp
    ros2_temp/raw_vision_packet_sub.cpp
    WindowEvaluator.cpp)


set(SOCCER_TEST_SRC
    BatteryProfileTest.cpp
    BallTest.cpp
    KickEvaluatorTest.cpp
    motion/TrapezoidalMotionTest.cpp
    optimization/GradientAscent1DTest.cpp
    optimization/ParallelGradientAscent1DTest.cpp
    optimization/NelderMead2DTest.cpp
    planning/tests/AnglePlanningTest.cpp
    planning/tests/BezierPathTest.cpp
    planning/tests/PlannerTest.cpp
    planning/tests/CreatePathTest.cpp
    planning/tests/TestingUtils.cpp
    planning/tests/TrajectoryTest.cpp
    planning/tests/TrapezoidalMotionTest.cpp
    planning/tests/VelocityProfilingTest.cpp
    TestMain.cpp
    vision/tests/CameraBallTest.cpp
    vision/tests/KalmanBallTest.cpp
    vision/tests/WorldBallTest.cpp
    vision/tests/CameraRobotTest.cpp
    vision/tests/KalmanRobotTest.cpp
    vision/tests/WorldRobotTest.cpp
    vision/tests/BallBounceTest.cpp
    vision/tests/CameraTest.cpp
    WindowEvaluatorTest.cpp
    LoggerTest.cpp)

set(LOG_VIEWER_RSRC
    ui/qt/log_icons.qrc)

set(LOG_VIEWER_UIS
    ui/qt/LogViewer.ui)

set(LOG_VIEWER_SRC
    LogViewer.cpp)

#======================================================================
# Add sources
#======================================================================
# ---- robocup ----
target_sources(
    robocup
    PRIVATE
        ${ROBOCUP_LIB_SRC}
        ${SOCCER_UIS}
        ${SOCCER_RSRC})

# Set the directory so AUTOUIC know where to find .ui files
set(UI_PATH ${CMAKE_CURRENT_SOURCE_DIR}/ui/qt)
set_target_properties(robocup PROPERTIES AUTOUIC_SEARCH_PATHS ${UI_PATH})


# ---- soccer ----
target_sources(
    soccer
    PRIVATE
        main.cpp
)

# ---- log_viewer ----
target_sources(
    log_viewer
    PRIVATE
        ${LOG_VIEWER_SRC}
        ${LOG_VIEWER_UIS}
        ${LOG_VIEWER_RSRC}
)

# Set the directory so AUTOUIC know where to find .ui files
set_target_properties(log_viewer PROPERTIES AUTOUIC_SEARCH_PATHS ${UI_PATH})

# ---- testing ----
if (BUILD_TESTING)
    target_sources(
        test-soccer
        PRIVATE
            ${SOCCER_TEST_SRC}
    )
endif()
