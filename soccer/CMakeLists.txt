
# Qt
set(CMAKE_AUTOMOC ON)

# include Eigen3 for linear algebra
include_directories(SYSTEM ${EIGEN_INCLUDE_DIR})

# All sources for the robocup library used for the python module, soccer, and log log viewer
# Note that main.cpp and LogViewer.cpp aren't present in this list because they're executable-specific
set(ROBOCUP_LIB_SRC
    "BatteryProfile.cpp"
    "Configuration.cpp"
    "gameplay/GameplayModule.cpp"
    "gameplay/robocup-py.cpp"
    "joystick/Joystick.cpp"
    "joystick/GamepadController.cpp"
    "joystick/GamepadJoystick.cpp"
    "joystick/SpaceNavJoystick.cpp"
    "KickEvaluator.cpp"
    "Logger.cpp"
    "motion/MotionControl.cpp"
    "motion/TrapezoidalMotion.cpp"
    "NewRefereeModule.cpp"
    "optimization/GradientAscent1D.cpp"
    "optimization/ParallelGradientAscent1D.cpp"
    "optimization/NelderMead2D.cpp"
    "optimization/PythonFunctionWrapper.cpp"
    "planning/CompositePath.cpp"
    "planning/DynamicObstacle.cpp"
    "planning/DirectTargetPathPlanner.cpp"
    "planning/TuningPathPlanner.cpp"
    "planning/EscapeObstaclesPathPlanner.cpp"
    "planning/InterpolatedPath.cpp"
    "planning/IndependentMultiRobotPathPlanner.cpp"
    "planning/InterceptPlanner.cpp"
    "planning/MotionConstraints.cpp"
    "planning/RotationConstraints.cpp"
    "planning/Path.cpp"
    "planning/RRTPlanner.cpp"
    "planning/RRTUtil.cpp"
    "planning/PivotPathPlanner.cpp"
    "planning/SettlePathPlanner.cpp"
    "planning/CollectPathPlanner.cpp"
    "planning/LineKickPlanner.cpp"
    "planning/SingleRobotPathPlanner.cpp"
    "planning/TargetVelPathPlanner.cpp"
    "planning/TrapezoidalPath.cpp"
    "Processor.cpp"
    "radio/NetworkRadio.cpp"
    "radio/PacketConvert.cpp"
    "radio/SimRadio.cpp"
    "Robot.cpp"
    "RobotConfig.cpp"
    "SystemState.cpp"
    "DebugDrawer.cpp"
    "ui/BatteryWidget.cpp"
    "ui/FieldView.cpp"
    "ui/MainWindow.cpp"
    "ui/ProtobufTree.cpp"
    "ui/RefereeTab.cpp"
    "ui/RobotStatusWidget.cpp"
    "ui/RobotWidget.cpp"
    "ui/SimFieldView.cpp"
    "ui/StripChart.cpp"
    "ui/StyleSheetManager.cpp"
    "VisionReceiver.cpp"
    "vision/ball/BallBounce.cpp"
    "vision/ball/CameraBall.cpp"
    "vision/ball/KalmanBall.cpp"
    "vision/ball/WorldBall.cpp"
    "vision/camera/Camera.cpp"
    "vision/camera/World.cpp"
    "vision/filter/KalmanFilter.cpp"
    "vision/filter/KalmanFilter2D.cpp"
    "vision/filter/KalmanFilter3D.cpp"
    "vision/kick/detector/FastKickDetector.cpp"
    "vision/kick/detector/SlowKickDetector.cpp"
    "vision/kick/estimator/ChipKickEstimator.cpp"
    "vision/kick/estimator/FlatKickEstimator.cpp"
    "vision/kick/KickEvent.cpp"
    "vision/robot/CameraRobot.cpp"
    "vision/robot/KalmanRobot.cpp"
    "vision/robot/WorldRobot.cpp"
    "vision/util/VisionFilterConfig.cpp"
    "vision/VisionFilter.cpp"
    "WindowEvaluator.cpp")


if(APPLE)
    # help CMake find gl.h, glu.h, etc on OS X
    link_directories(/opt/X11/lib/)
    include_directories(SYSTEM /opt/X11/include/)
    # lib dir for spnav
    link_directories(/usr/local/lib/)
endif()

include_directories("${CMAKE_CURRENT_SOURCE_DIR}")
include_directories(${CMAKE_CURRENT_BINARY_DIR})

file(GLOB SOCCER_UIS "${CMAKE_CURRENT_SOURCE_DIR}/ui/qt/*.ui")
qt5_wrap_ui(SOCCER_UIS ${SOCCER_UIS})

qt5_add_resources(SOCCER_RSRC ui/qt/main_icons.qrc)


# Boost Python
set(Boost_USE_STATIC_LIBS   OFF)
set(Boost_USE_MULTITHREADED ON)
set(Boost_USE_STATIC_RUNTIME OFF)
find_package(Boost COMPONENTS system REQUIRED)

# find boost python
# This package is named 'python3' on Arch, 'python-py34' on Ubuntu 14, and 'python' on OS X
set(FOUND_A_BOOST_PYTHON FALSE)
# Find the matching boost python implementation
set(version ${PYTHONLIBS_VERSION_STRING})

if(APPLE)
    set(boost_py_version "")
else()
    STRING( REGEX REPLACE "[^0-9]" "" boost_py_version "${version}" )
    # Get first two numbers of python string
    STRING( REGEX MATCH "^[0-9][0-9]" boost_py_version "${boost_py_version}" )
endif()

foreach(possible_name python3 python-py${boost_py_version} python)
    find_package(Boost COMPONENTS ${possible_name} QUIET)
    if (Boost_FOUND)
        set(FOUND_A_BOOST_PYTHON TRUE)
        break()
    endif()
endforeach()
if (NOT FOUND_A_BOOST_PYTHON)
    message(FATAL_ERROR "Unable to find a suitable version of boost python")
endif()

# libusb
include_directories(SYSTEM ${LIBUSB_1_INCLUDE_DIRS})

# SDL
include_directories(SYSTEM ${SDL2_INCLUDE_DIRS})

# Build stand-alone soccer dylib
# This is linked into soccer and our unit tests, as well as being a python module
add_library(robocup SHARED ${ROBOCUP_LIB_SRC} ${SOCCER_UIS} ${SOCCER_RSRC})
set_target_properties(robocup PROPERTIES PREFIX "")
set_target_properties(robocup PROPERTIES SUFFIX ".so")

target_link_libraries(robocup common rc-fshare)
qt5_use_modules(robocup Widgets Xml Core OpenGL Network Svg)
target_link_libraries(robocup ${LIBUSB_1_LIBRARIES})
target_link_libraries(robocup ${Boost_LIBRARIES} ${Boost_SYSTEM_LIBRARY})
target_link_libraries(robocup GL GLU glut)
target_link_libraries(robocup pthread)
target_link_libraries(robocup spnav)
target_link_libraries(robocup ${SDL2_LIBRARIES})


# RRT
include_directories(${RRT_INCLUDE_DIR})
target_link_libraries(robocup RRT)


# python
# note: these are set in the root CMakeLists.txt file
include_directories(SYSTEM ${PYTHON_INCLUDE_DIRS})
link_directories(${PYTHON_LINK_DIRS})
target_link_libraries(robocup ${PYTHON_LIBRARIES})

# 'soccer' program
if(APPLE)
    include(BundleUtilities)

    # app icon
    set(OSX_ICON_FILES ${CMAKE_CURRENT_SOURCE_DIR}/ui/qt/SoccerBall.icns)
    set_source_files_properties(${OSX_ICON_FILES} PROPERTIES MACOSX_PACKAGE_LOCATION Resources)
    set(MACOSX_BUNDLE_ICON_FILE SoccerBall)

    set(MACOSX_BUNDLE_GUI_IDENTIFIER "org.robojackets.robocup.soccer")

    add_executable(soccer MACOSX_BUNDLE main.cpp ${OSX_ICON_FILES})

    # add a script called "soccer" in the run/ dir that launches the soccer app and passes on all args,
    # so it can be used the same way the soccer executable is used on Linux
    file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/soccer
        "#!/bin/sh\n"
        "$(dirname $0)/soccer.app/Contents/MacOS/soccer $@")
    file(INSTALL ${CMAKE_CURRENT_BINARY_DIR}/soccer
        DESTINATION ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}
        FILE_PERMISSIONS OWNER_READ GROUP_READ OWNER_WRITE GROUP_WRITE OWNER_EXECUTE GROUP_EXECUTE WORLD_EXECUTE)
else()
    add_executable(soccer main.cpp)
endif()
qt5_use_modules(soccer Widgets Xml Core OpenGL Network Svg)
target_link_libraries(soccer robocup)

# build the 'log_viewer' program
qt5_add_resources(LOG_VIEWER_RSRC ui/qt/log_icons.qrc)
qt5_wrap_ui(LOG_VIEWER_UI ui/qt/LogViewer.ui)
set(LOG_VIEWER_SRC
    "LogViewer.cpp"
)
add_executable(log_viewer ${LOG_VIEWER_SRC} ${LOG_VIEWER_UI} ${LOG_VIEWER_RSRC})
qt5_use_modules(log_viewer Core Widgets OpenGL Svg Xml)
target_link_libraries(log_viewer robocup)


# Add a test runner target "test-soccer" to run all tests in this directory
set(SOCCER_TEST_SRC
    "${CMAKE_SOURCE_DIR}/common/Geometry2d/LineTest.cpp"
    "${CMAKE_SOURCE_DIR}/common/Geometry2d/PointTest.cpp"
    "${CMAKE_SOURCE_DIR}/common/Geometry2d/RectTest.cpp"
    "${CMAKE_SOURCE_DIR}/common/Geometry2d/SegmentTest.cpp"
    "${CMAKE_SOURCE_DIR}/common/Geometry2d/CompositeShapeTest.cpp"
    "${CMAKE_SOURCE_DIR}/common/Geometry2d/ArcTest.cpp"
    "${CMAKE_SOURCE_DIR}/common/Geometry2d/TransformMatrixTest.cpp"
    "${CMAKE_SOURCE_DIR}/common/Geometry2d/PoseTest.cpp"
    "BatteryProfileTest.cpp"
    "KickEvaluatorTest.cpp"
    "motion/TrapezoidalMotionTest.cpp"
    "optimization/GradientAscent1DTest.cpp"
    "optimization/ParallelGradientAscent1DTest.cpp"
    "optimization/NelderMead2DTest.cpp"
    "planning/PathTest.cpp"
    "planning/EscapeObstaclesPathPlannerTest.cpp"
    "planning/TargetVelPathPlannerTest.cpp"
    "TestMain.cpp"
    "vision/tests/CameraBallTest.cpp"
    "vision/tests/KalmanBallTest.cpp"
    "vision/tests/WorldBallTest.cpp"
    "vision/tests/CameraRobotTest.cpp"
    "vision/tests/KalmanRobotTest.cpp"
    "vision/tests/WorldRobotTest.cpp"
    "vision/tests/BallBounceTest.cpp"
    "vision/tests/CameraTest.cpp"
    "WindowEvaluatorTest.cpp"
)
add_executable(test-soccer ${SOCCER_TEST_SRC})
target_link_libraries(test-soccer robocup)
qt5_use_modules(test-soccer Core Widgets Xml)
target_link_libraries(test-soccer ${GTEST_LIBRARIES})
add_dependencies(test-soccer googletest)

# Don't build the tests by default
set_target_properties(test-soccer PROPERTIES EXCLUDE_FROM_ALL TRUE)
