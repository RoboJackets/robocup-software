project (motion)
cmake_minimum_required(VERSION 2.6)

# always use all warnings and warnings as errors
add_definitions (-Werror -Wall)

# disables use of lowercase slot/signals keywords
# instead use Q_SLOTS/Q_SIGNALS
#add_definitions (-DQT_NO_KEYWORDS)

# include binary directory for Qt generated files
include_directories (${PROJECT_BINARY_DIR})

#uncomment to link to all the stuff in common
set (USE_NETWORK true)
set (USE_GEOMETRY true)
set (USE_FRAMEWORK true)
include ("${PROJECT_SOURCE_DIR}/../common/CMakeLists.txt")

# uncomment to disable the qt gui classes
# useful for command line only applications
# set (QT_USE_QTGUI false)

# uncomment to enable Qt network classes
# set (QT_USE_QTNETWORK true)

# uncomment to enable Qt opengl classes
# set (QT_USE_QTOPENGL true)

set (QT_USE_QTXML true)

# this command finds the Qt4 libraries and sets all required variables
find_package ( Qt4 REQUIRED )
# include the Qt file with macros and constants defined
include ( ${QT_USE_FILE} )

# this will run MOC on the specified header (.hpp) files
# needed for any headers that use signals/slots
qt4_wrap_cpp (MOC_SRCS
	MainWindow.hpp
        FieldDisplay.hpp
)

# this will run UIC on the .ui files
qt4_wrap_ui ( UI_HDRS
	ui/motion.ui
)

# include any Qt resource files
qt4_add_resources ( RC_SRCS
)

# add all .cpp/.c sources here
set (SRCS
	main.cpp
	MainWindow.cpp
        FieldDisplay.cpp
        Graphikos/Field.cpp
        RobotPath.cpp
        Motion.cpp
        Robot.cpp
        Pid.cpp
        ConfigFile.cpp
)

# tell cmake to create an executable
add_executable (motion ${SRCS} ${MOC_SRCS} ${UI_HDRS})

# link against qt libraries
target_link_libraries (motion ${QT_LIBRARIES} ${GEOMETRY_LIBRARY} ${NETWORK_LIBRARY})

target_link_libraries(motion ${FRAMEWORK_LIBRARY})