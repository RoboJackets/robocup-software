project (referee)

add_definitions (-Werror)

## disable the qt language extensions in favor of Q_SLOTS...etc
add_definitions (-DQT_NO_KEYWORDS)

set (USE_PACKET true)
include ("../common/CMakeLists.txt")


# needed for qt ui generated header files
include_directories (${PROJECT_BINARY_DIR})
include_directories (${PROJECT_SOURCE_DIR})

# enable QT networking features
set (QT_USE_QTNETWORK true)

# this command finds the Qt4 libraries and sets all required variables
find_package ( Qt4 REQUIRED )

# add some useful macros and variables
# QT_USE_FILE is a variable defined by FIND_PACKAGE( Qt4 ). 
# It contains a path to CMake scripts
include ( ${QT_USE_FILE} )

# add all source files here
SET( SRCS
	main.cpp
	Ref.cpp
	)

# this will run MOC on the specified header files
qt4_wrap_cpp ( MOC_SRCS
	)
	
# this will run UIC on the .ui files
qt4_wrap_ui ( UI_HDRS
	)
	
# list any resource files
qt4_add_resources ( RC_SRCS
	)

add_executable (${PROJECT_NAME} ${SRCS} ${MOC_SRCS} ${UI_HDRS})

#FIND_PACKAGE prepared QT_LIBRARIES variable for us:
target_link_libraries (${PROJECT_NAME} ${QT_LIBRARIES} ${PACKET})
