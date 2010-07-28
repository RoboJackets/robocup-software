project (Soccer)
cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

add_definitions(-O2 -Wall -g3)

# gtsam include
include_directories ( ${PROJECT_SOURCE_DIR}/../packages/gtsam_install/include )
include_directories ( ${PROJECT_SOURCE_DIR}/../packages/gtsam/CppUnitLite )
link_directories ( ${PROJECT_SOURCE_DIR}/../packages/gtsam_install/lib )

include (${PROJECT_SOURCE_DIR}/../common/CMakeLists.txt)

include_directories(${PROJECT_BINARY_DIR})

set (SRCS
	main.cpp
	Processor.cpp
	Logger.cpp
	MainWindow.cpp
	ProtobufTree.cpp
	Joystick.cpp
	RefereeModule.cpp
	PlayConfigTab.cpp
	FieldView.cpp
	SimFieldView.cpp

	../common/Utils.cpp
	../common/multicast.cpp
	
	# config file tab components
	configuration/ConfigFileTab.cpp
	configuration/ConfigFileModel.cpp
	configuration/ConfigFileItem.cpp

	framework/ConfigFile.cpp
	framework/Obstacle.cpp
	framework/Path.cpp
	framework/SystemState.cpp

# Sources for gameplay module

	# Core gameplay components
	gameplay/GameplayModule.cpp
	gameplay/Play.cpp
	gameplay/Behavior.cpp
	gameplay/Window.cpp
	gameplay/Robot.cpp
	gameplay/PreventDoubleTouch.cpp
	
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
	#gameplay/plays/AggressiveZoneOffense.cpp

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
	#gameplay/plays/test_plays/TestBallSpeed.cpp
	#gameplay/plays/test_plays/TestBasicChipAttack.cpp #disabled until 2010 robots ready
	#gameplay/plays/test_plays/TestBasicOneTouchChipAttack.cpp
	#gameplay/plays/test_plays/TestKicks.cpp
	
	# Demo plays 
	#gameplay/plays/challenges/MixedChallenge2010.cpp
	
	# files that use the optimizer - may want to disable
	#gameplay/plays/test_plays/TestPassConfigOptimize.cpp
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

qt4_automoc (${SRCS})

qt4_wrap_ui (UI_HDRS
	ui/MainWindow.ui
	ui/PlayConfigTab.ui
	ui/ConfigFileTab.ui
)

qt4_add_resources (RC_FILES
	ui/main_icons.qrc
	ui/log_icons.qrc
)

add_executable (soccer ${SRCS} ${UI_HDRS} ${RC_FILES})

target_link_libraries(soccer ${GEOMETRY_LIBRARY})
target_link_libraries(soccer ${CONFIG_LIBRARY})
target_link_libraries(soccer blas lapack)
target_link_libraries(soccer packets)
target_link_libraries(soccer ${QT_LIBRARIES})
target_link_libraries(soccer gtsam)

# Enable this for Google performance tools
#target_link_libraries(soccer tcmalloc)

# Log viewer
qt4_wrap_ui (LOG_UI_HDRS ui/LogViewer.ui)

set(LOG_SRCS
	LogViewer.cpp
	FieldView.cpp
	ProtobufTree.cpp
)

qt4_automoc(${LOG_SRCS})
qt4_add_resources(LOG_RCS ui/log_icons.qrc)

add_executable(log_viewer ${LOG_SRCS} ${LOG_UI_HDRS} ${LOG_RCS})
target_link_libraries(log_viewer ${QT_LIBRARIES} packets)

# Simple logger
add_executable(simple_logger
	simple_logger.cpp
	../common/multicast.cpp
	../common/Utils.cpp
)
target_link_libraries(simple_logger ${QT_LIBRARIES} packets)

# tcpdump log converter
add_executable(convert-tcpdump
	convert-tcpdump.cpp
)
target_link_libraries(convert-tcpdump packets pthread pcap)
