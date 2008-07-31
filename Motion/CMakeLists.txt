project (motion)

cmake_minimum_required(VERSION 2.6)

add_definitions (-Werror -Wall -O2 -funroll-loops)
add_definitions (-DQT_NO_KEYWORDS)

set (USE_PACKET true)
set (USE_GEOMETRY true)
include ("../common/CMakeLists.txt")

include_directories (${PROJECT_BINARY_DIR})

set (QT_USE_QTGUI false)
set (QT_USE_QTNETWORK true)
set (QT_USE_QTXML true)

# this command finds the Qt4 libraries and sets all required variables
find_package ( Qt4 REQUIRED )

# add some useful macros and variables
# QT_USE_FILE is a variable defined by FIND_PACKAGE( Qt4 ). 
# It contains a path to CMake scripts
include ( ${QT_USE_FILE} )

# add all source files here
SET( SRCS
	main.cpp
	MotionController.cpp
	InputHandler.cpp
	JoystickInput.cpp
	Pid.cpp
	PathPlanner.cpp
	SimplePlanner.cpp
	ConfigFile.cpp
	Robot.cpp
	)

# this will run MOC on the specified header files
qt4_wrap_cpp ( MOC_SRCS
    InputHandler.hpp
    MotionController.hpp
	)
	
# this will run UIC on the .ui files
qt4_wrap_ui ( UI_HDRS
	)
	
# list any resource files
qt4_add_resources ( RC_SRCS
	)

add_executable (${PROJECT_NAME} ${SRCS} ${MOC_SRCS} ${UI_HDRS}) 
add_dependencies (${PROJECT_NAME} GeneratePackets)

#FIND_PACKAGE prepared QT_LIBRARIES variable for us:
target_link_libraries (${PROJECT_NAME} ${QT_LIBRARIES} ${PACKET} ${GEOMETRY})
