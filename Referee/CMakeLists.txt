project (QtGuiExample)
cmake_minimum_required(VERSION 2.6)

# always use all warnings and warnings as errors
add_definitions (-Werror -Wall)

# disables use of lowercase slot/signals keywords
# instead use Q_SLOTS/Q_SIGNALS
add_definitions (-DQT_NO_KEYWORDS)

# include binary directory for Qt generated files
include_directories (${PROJECT_BINARY_DIR})

# uncomment to disable the qt gui classes
# useful for command line only applications
# set (QT_USE_QTGUI false)

# uncomment to enable Qt network classes
# set (QT_USE_QTNETWORK true)

# uncomment to enable Qt opengl classes
# set (QT_USE_QTOPENGL true)

# this command finds the Qt4 libraries and sets all required variables
find_package ( Qt4 REQUIRED )
# include the Qt file with macros and constants defined
include ( ${QT_USE_FILE} )

# this will run MOC on the specified header (.hpp) files
# needed for any headers that use signals/slots
qt4_wrap_cpp (MOC_SRCS
	Referee.hpp
)
	
# this will run UIC on the .ui files
qt4_wrap_ui ( UI_HDRS
	ui/Referee.ui
)
	
# include any Qt resource files
qt4_add_resources ( RC_SRCS
	ui/resource.qrc
)

# add all .cpp/.c sources here
set (SRCS
	main.cpp
	Referee.cpp
)

# tell cmake to create an executable
add_executable (sslreferee ${SRCS} ${MOC_SRCS} ${UI_HDRS} ${RC_SRCS})

# link against qt libraries
target_link_libraries (sslreferee ${QT_LIBRARIES})