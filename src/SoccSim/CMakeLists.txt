project (soccsim)

add_definitions (-Werror -Wall)
add_definitions (-DQT_NO_KEYWORDS)

set (USE_PACKET true)
set (USE_GEOMETRY true)
include ("../common/CMakeLists.txt")

## makes qt ui header includes work
include_directories(${PROJECT_SOURCE_DIR})
include_directories(${PROJECT_BINARY_DIR})

set (QT_USE_QTNETWORK true)
set (QT_USE_QTOPENGL true)

# this command finds the Qt4 libraries and sets all required variables
find_package ( Qt4 REQUIRED )

# add some useful macros and variables
# QT_USE_FILE is a variable defined by FIND_PACKAGE( Qt4 ). 
# It contains a path to CMake scripts
include ( ${QT_USE_FILE} )

# add all source files here
SET( SRCS
	main.cpp
	Viewer.cpp
	Vision.cpp
	SoccSimWindow.cpp
	
	Physics/Env.cpp
	Physics/Ball.cpp
	Physics/Field.cpp
	Physics/Robot.cpp
	Physics/Entity.cpp
	)

# this will run MOC on the specified header files
qt4_wrap_cpp ( MOC_SRCS
	SoccSimWindow.hpp
	)
	
# this will run UIC on the .ui files
qt4_wrap_ui ( UI_HDRS
	ui/SimulatorWindow.ui
	)
	
# list any resource files
qt4_add_resources ( RC_SRCS
	)
	
add_executable (${PROJECT_NAME} ${SRCS} ${MOC_SRCS} ${UI_HDRS})

#FIND_PACKAGE prepared QT_LIBRARIES variable for us:
target_link_libraries (${PROJECT_NAME} ${QT_LIBRARIES} ode ${PACKET} ${GEOMETRY})
