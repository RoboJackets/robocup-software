project (logger)

add_definitions (-Werror)
add_definitions (-DQT_NO_KEYWORDS)

set (USE_PACKET true)
set (USE_GEOMETRY true)
include ("../common/CMakeLists.txt")

# needed for qt ui generated header files
include_directories (${PROJECT_BINARY_DIR})
include_directories (${PROJECT_SOURCE_DIR})

# add all source files here
SET( SRCS
	main.cpp
	MainWindow.cpp
	FieldDisplay.cpp
	PlaybackControl.cpp
	Tools.cpp
	
	Log/Logger.cpp
	Log/LiveOnly.cpp
	Log/Packet.cpp
	
	#packets
	Log/Packets/VisionData.cpp
	Log/Packets/LogMotion.cpp
	Log/Packets/CommData.cpp
	#Log/Packets/MotionCmd.cpp
	Log/Packets/RobotStatus.cpp
	Log/Packets/Ref.cpp
	
	Graphikos/Ball.cpp
	Graphikos/Robot.cpp
	Graphikos/Field.cpp
	)

# this will run MOC on the specified header files
qt4_wrap_cpp ( MOC_SRCS
	MainWindow.hpp
	PlaybackControl.hpp
	FieldDisplay.hpp
	Tools.hpp
	)
	
# this will run UIC on the .ui files
qt4_wrap_ui ( UI_HDRS
	ui/MainWindow.ui
	)
	
# list any resource files
qt4_add_resources ( RC_SRCS
	)

add_executable (${PROJECT_NAME} ${SRCS} ${MOC_SRCS} ${UI_HDRS})

#FIND_PACKAGE prepared QT_LIBRARIES variable for us:
target_link_libraries (${PROJECT_NAME} ${QT_LIBRARIES} ${PACKET} ${GEOMETRY})
