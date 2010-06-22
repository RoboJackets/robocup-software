project (Soccer)
cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

add_definitions(-O2 -fno-inline -Wall -g3)

set (USE_QT true)
set (USE_NETWORK true)
set (USE_GEOMETRY true)
set (USE_FRAMEWORK true)
set (USE_CONFIG true)

# setup gtsam inclusion
# add_custom_command( OUTPUT gtsam_configure
#					COMMAND ${PROJECT_SOURCE_DIR}/packages/

# gtsam include
include_directories ( ${PROJECT_SOURCE_DIR}/../packages/gtsam_install/include )
include_directories ( ${PROJECT_SOURCE_DIR}/../packages/gtsam/CppUnitLite )
link_directories ( ${PROJECT_SOURCE_DIR}/../packages/gtsam_install/lib )

include ("${PROJECT_SOURCE_DIR}/../common/CMakeLists.txt")
add_subdirectory (${PROJECT_SOURCE_DIR}/../common/vision ${PROJECT_BINARY_DIR}/vision)
include_directories (${PROJECT_SOURCE_DIR}/..)
include_directories (${PROJECT_SOURCE_DIR}/modeling/)
include_directories (${PROJECT_SOURCE_DIR}/gameplay/)
include_directories (${PROJECT_SOURCE_DIR}/gameplay/optimizer)
add_subdirectory (${PROJECT_SOURCE_DIR}/../log ${PROJECT_BINARY_DIR}/log)

include_directories(${PROJECT_BINARY_DIR})
	
set (SRCS
	main.cpp
	Processor.cpp
	MainWindow.cpp
	JoystickInput.cpp
	RefereeModule.cpp
	PlayConfigTab.cpp
	
	# config file tab components
	configuration/ConfigFileTab.cpp
	configuration/ConfigFileModel.cpp
	configuration/ConfigFileItem.cpp

# Sources for gameplay module

	# Core gameplay components
	gameplay/GameplayModule.cpp
	gameplay/Play.cpp
	gameplay/Behavior.cpp
	gameplay/Window.cpp
	gameplay/Robot.cpp
	
	# Optimization components
	gameplay/optimizer/PassConfig.cpp
	gameplay/optimizer/PassState.cpp
	gameplay/optimizer/AnalyticPassPlanner.cpp
	gameplay/optimizer/passOptimization.cpp
	gameplay/optimizer/DrivingFactors.cpp
	gameplay/optimizer/PassingFactors.cpp
	gameplay/optimizer/ShootingFactors.cpp
	
	# Main Plays
	gameplay/plays/OurKickoff.cpp
	gameplay/plays/TheirKickoff.cpp
	gameplay/plays/OurFreekick.cpp
	gameplay/plays/TheirFreekick.cpp
	gameplay/plays/KickPenalty.cpp
	gameplay/plays/DefendPenalty.cpp
	gameplay/plays/Stopped.cpp
	gameplay/plays/Offense.cpp
	gameplay/plays/ClearBall.cpp
	gameplay/plays/Defense.cpp
	gameplay/plays/DefendGoal.cpp
	gameplay/plays/OptimizedOffense.cpp
	gameplay/plays/AggressiveZoneOffense.cpp

    # Core behaviors
	gameplay/behaviors/Move.cpp
	gameplay/behaviors/Kickoff.cpp
	gameplay/behaviors/Penalty.cpp
	gameplay/behaviors/Idle.cpp
	gameplay/behaviors/Kick.cpp
	gameplay/behaviors/OneTouchKick.cpp
	gameplay/behaviors/Intercept.cpp
	gameplay/behaviors/OptimizedPassing.cpp
	gameplay/behaviors/Mark.cpp
	gameplay/behaviors/GoalDefender.cpp
	gameplay/behaviors/ZoneOffense.cpp
	
	# Positions
	gameplay/behaviors/positions/Goalie.cpp      
	gameplay/behaviors/positions/Fullback.cpp
 	gameplay/behaviors/positions/Forward.cpp
 	
	# testing plays/behaviors
	gameplay/plays/test_plays/TestBasicPassing.cpp
	gameplay/plays/test_plays/TestBasicOneTouchPassing.cpp
	gameplay/plays/test_plays/TestDirectMotionControl.cpp
	gameplay/plays/test_plays/TestRectMotionControl.cpp
	gameplay/plays/test_plays/TestTimePositionControl.cpp
	gameplay/plays/test_plays/TestBasicAttack.cpp
	gameplay/plays/test_plays/TestBasicOneTouchAttack.cpp
	gameplay/plays/test_plays/TestIntercept.cpp
	gameplay/plays/test_plays/TestPassPlay.cpp
	gameplay/plays/test_plays/TestOptimizedPassPlay.cpp
	gameplay/plays/test_plays/TestGUI.cpp
	gameplay/plays/test_plays/TestBallSpeed.cpp
	#gameplay/plays/test_plays/TestBasicChipAttack.cpp #disabled until 2010 robots ready
	#gameplay/plays/test_plays/TestBasicOneTouchChipAttack.cpp
	#gameplay/plays/test_plays/TestKicks.cpp
	
	# files that use the optimizer - may want to disable
	gameplay/plays/test_plays/TestPassConfigOptimize.cpp
	gameplay/optimizer/PassOptimizer.cpp

	# files for pass experiments
	gameplay/plays/test_plays/TestPassExperiment1.cpp

# 	gameplay/behaviors/test/KickerCalibration.cpp   
# 	gameplay/behaviors/test/MotionLeader.cpp    

# Sources for modeling module
	modeling/WorldModel.cpp
	modeling/BallModel.cpp
	modeling/RobotModel.cpp
	modeling/BLASWrap/blaswrap.cpp
	modeling/difference_kalman.cpp
	modeling/Rbpf.cpp
	modeling/RbpfModel.cpp
	modeling/RbpfModelGraph.cpp
	modeling/RbpfState.cpp
	modeling/InvertMatrix.hpp
	modeling/RbpfModelRolling.cpp
	modeling/RbpfModelKicked.cpp
	
# Sources for motion module
	motion/MotionModule.cpp
	motion/Robot.cpp
	motion/Pid.cpp
	motion/Dynamics.cpp
	motion/planning/rrt.cpp
	motion/planning/Tree.cpp

# Sources for stateID module
	stateID/StateIDModule.cpp
)

qt4_wrap_cpp ( MOC_SRCS
	motion/MotionModule.hpp
)

qt4_automoc (${SRCS})

qt4_wrap_ui (UI_HDRS
	ui/MainWindow.ui
	ui/RefereeTab.ui
	ui/PlayConfigTab.ui
	ui/ConfigFileTab.ui
	motion/ui/config.ui
)

qt4_add_resources (RC_FILES
	ui/ref_icons.qrc
	ui/main_icons.qrc
)

set (target soccer)
add_executable (${target} ${SRCS} ${MOC_SRCS} ${UI_HDRS} ${RC_FILES})

target_link_libraries(${target} ${PACKET_LIBRARY})
target_link_libraries(${target} ${FRAMEWORK_LIBRARY})
target_link_libraries(${target} ${GEOMETRY_LIBRARY})
target_link_libraries(${target} ${NETWORK_LIBRARY})
target_link_libraries(${target} ${CONFIG_LIBRARY})
target_link_libraries(${target} blas lapack)
target_link_libraries(${target} vision)
target_link_libraries(${target} ${QT_LIBRARIES})
target_link_libraries(${target} gtsam ) # libraries required by gtsam


#Module Linking
# Add a new line here for every module used
target_link_libraries(${target} log)
