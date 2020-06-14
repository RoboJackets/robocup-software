#======================================================================
# Set Sources
#======================================================================
set(SRC_PREFIX ${CMAKE_CURRENT_SOURCE_DIR})

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
    planning/paths/CompositePath.cpp
    planning/DynamicObstacle.cpp
    planning/planners/DirectTargetPathPlanner.cpp
    planning/planners/TuningPathPlanner.cpp
    planning/planners/EscapeObstaclesPathPlanner.cpp
    planning/paths/InterpolatedPath.cpp
    planning/planners/IndependentMultiRobotPathPlanner.cpp
    planning/planners/InterceptPlanner.cpp
    planning/MotionConstraints.cpp
    planning/RotationConstraints.cpp
    planning/paths/Path.cpp
    planning/trajectory/Trajectory.cpp
    planning/planners/RRTPlanner.cpp
    planning/planners/RRTUtil.cpp
    planning/planners/PivotPathPlanner.cpp
    planning/planners/SettlePathPlanner.cpp
    planning/planners/CollectPathPlanner.cpp
    planning/planners/LineKickPlanner.cpp
    planning/planners/SingleRobotPathPlanner.cpp
    planning/planners/TargetVelPathPlanner.cpp
    planning/PlannerNode.cpp
    planning/paths/TrapezoidalPath.cpp
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
    VisionReceiver.cpp
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
    WindowEvaluator.cpp)


set(SOCCER_TEST_SRC
    ${CMAKE_SOURCE_DIR}/common/Geometry2d/ArcTest.cpp
    ${CMAKE_SOURCE_DIR}/common/Geometry2d/CircleTest.cpp
    ${CMAKE_SOURCE_DIR}/common/Geometry2d/CompositeShapeTest.cpp
    ${CMAKE_SOURCE_DIR}/common/Geometry2d/LineTest.cpp
    ${CMAKE_SOURCE_DIR}/common/Geometry2d/PointTest.cpp
    ${CMAKE_SOURCE_DIR}/common/Geometry2d/PoseTest.cpp
    ${CMAKE_SOURCE_DIR}/common/Geometry2d/RectTest.cpp
    ${CMAKE_SOURCE_DIR}/common/Geometry2d/SegmentTest.cpp
    ${CMAKE_SOURCE_DIR}/common/Geometry2d/TransformMatrixTest.cpp
    BatteryProfileTest.cpp
    KickEvaluatorTest.cpp
    motion/TrapezoidalMotionTest.cpp
    optimization/GradientAscent1DTest.cpp
    optimization/ParallelGradientAscent1DTest.cpp
    optimization/NelderMead2DTest.cpp
    planning/PathTest.cpp
    planning/planners/EscapeObstaclesPathPlannerTest.cpp
    planning/planners/TargetVelPathPlannerTest.cpp
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

#qt5_add_resources(LOG_VIEWER_RSRC ui/qt/log_icons.qrc)
#qt5_wrap_ui(LOG_VIEWER_UI ui/qt/LogViewer.ui)
#set(LOG_VIEWER_SRC
#    LogViewer.cpp)

#======================================================================
# Add sources
#======================================================================
target_sources(
    robocup
    PRIVATE
        ${ROBOCUP_LIB_SRC}
        ${SOCCER_UIS}
        ${SOCCER_RSRC})

#target_sources(
#    log_viewer
#    PRIVATE
#        ${LOG_VIEWER_SRC}
#        ${LOG_VIEWER_UI}
#        ${LOG_VIEWER_RSRC})
#
