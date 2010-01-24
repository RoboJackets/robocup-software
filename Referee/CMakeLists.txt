project (QtGuiExample)
cmake_minimum_required(VERSION 2.6)

# always use all warnings and warnings as errors
add_definitions (-Werror -Wall )
add_definitions (-g `pkg-config gtkmm-2.4 --cflags`)

INCLUDE(FindPkgConfig)
PKG_CHECK_MODULES(GTKMM gtkmm-2.4)

IF(GTKMM_FOUND)
	SET(K3D_GTKMM_INCLUDE_DIRS
		${GTKMM_INCLUDE_DIRS}
		)

	SET(K3D_GTKMM_LIB_DIRS
		${GTKMM_LIBRARY_DIRS}
		)

	SET(K3D_GTKMM_LIBS
		${GTKMM_LIBRARIES}
		)

	SET(K3D_GTKMM_FOUND 1)
ENDIF(GTKMM_FOUND)

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
        Commands.hpp
 	Serial.hpp
	UDP_Broadcast.hpp
	GameControl.hpp
	GameInfo.hpp
)
	
# this will run UIC on the .ui files
qt4_wrap_ui ( UI_HDRS
	ui/MainRef.ui
)
	
# include any Qt resource files
qt4_add_resources ( RC_SRCS
	ui/resource.qrc
)

# add all .cpp/.c sources here
set (SRCS
	main.cpp
	Referee.cpp
	Serial.cpp
	UDP_Broadcast.cpp
	GameControl.cpp
	GameInfo.cpp
)

include_directories( ${GTKMM_INCLUDE_DIRS}  )
link_directories( ${GTKMM_LIBRARY_DIRS}  )

# tell cmake to create an executable
add_executable (sslreferee ${SRCS} ${MOC_SRCS} ${UI_HDRS} ${RC_SRCS})

# link against qt libraries
target_link_libraries (sslreferee ${GTKMM_LIBRARIES} ${QT_LIBRARIES})

# Copy the executable out to project /bin folder
#file(COPY ./bin/sslreferee DESTINATION ../../bin/)
