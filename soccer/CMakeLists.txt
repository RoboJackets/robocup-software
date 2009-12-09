project (Soccer)
cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

add_definitions(-O2 -fno-inline -Wall -g3)

set (USE_QT true)
set (USE_NETWORK true)
set (USE_GEOMETRY true)
set (USE_FRAMEWORK true)
set (USE_CONFIG true)

include ("${PROJECT_SOURCE_DIR}/../common/CMakeLists.txt")
add_subdirectory (${PROJECT_SOURCE_DIR}/../common/vision ${PROJECT_BINARY_DIR}/vision)

include_directories (${PROJECT_SOURCE_DIR}/..)
include_directories (${PROJECT_SOURCE_DIR}/modeling/)
add_subdirectory (${PROJECT_SOURCE_DIR}/../log ${PROJECT_BINARY_DIR}/log)

include_directories(${PROJECT_BINARY_DIR})

set (SRCS
	main.cpp
	Processor.cpp
	MainWindow.cpp
	InputHandler.cpp
	JoystickInput.cpp
	RefereeModule.cpp
	PlayConfigTab.cpp

# Sources for gameplay module

	gameplay/GameplayModule.cpp
	gameplay/Play.cpp
	gameplay/Behavior.cpp
	gameplay/Window.cpp
	gameplay/Robot.cpp
	
	# Main Plays
	gameplay/plays/OurKickoff.cpp
	gameplay/plays/TheirKickoff.cpp
	gameplay/plays/OurFreekick.cpp
	gameplay/plays/TheirFreekick.cpp
	gameplay/plays/KickPenalty.cpp
	gameplay/plays/DefendPenalty.cpp
	gameplay/plays/Stopped.cpp
	gameplay/plays/Offense.cpp
	gameplay/plays/Defense.cpp
	gameplay/plays/OptimizedOffense.cpp
	gameplay/plays/OktayPlay.cpp

	gameplay/behaviors/Move.cpp
	gameplay/behaviors/Kickoff.cpp
	gameplay/behaviors/Penalty.cpp
	gameplay/behaviors/Idle.cpp
	gameplay/behaviors/Kick.cpp
	gameplay/behaviors/Intercept.cpp
	gameplay/behaviors/positions/Goalie.cpp      
	gameplay/behaviors/positions/Fullback.cpp
 	gameplay/behaviors/positions/Forward.cpp
 	
	# testing plays/behaviors
	gameplay/plays/test_plays/TestBasicPassing.cpp
	gameplay/plays/test_plays/TestDirectMotionControl.cpp
	gameplay/plays/test_plays/TestTimePositionControl.cpp
	gameplay/plays/test_plays/TestBasicAttack.cpp
	gameplay/behaviors/test/Passer.cpp
	gameplay/behaviors/test/Receiver.cpp
	
	gameplay/plays/test_plays/TestPassPlay.cpp
	gameplay/plays/test_plays/PassConfig.cpp
	gameplay/plays/test_plays/PassState.cpp
	gameplay/plays/test_plays/PassCommand.cpp

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
	modeling/RbpfAllModels.cpp
	
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

#Module Linking
# Add a new line here for every module used
target_link_libraries(${target} log)
