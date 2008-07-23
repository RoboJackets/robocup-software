project (grab)

set (USE_PACKET true)
set (USE_GEOMETRY true)
include ("../../common/CMakeLists.txt")

add_definitions (-Werror)
add_definitions (-DQT_NO_KEYWORDS)
set (QT_USE_QTNETWORK true)
find_package (Qt4 REQUIRED)
include (${QT_USE_FILE})

include_directories (${PROJECT_BINARY_DIR})

# add all source files here
SET (SRCS
	grab.cpp Grab_Window.cpp Record.cpp Receive_Thread.cpp
	)

qt4_wrap_cpp (MOC_SRCS
    Grab_Window.hpp
    )
	
qt4_wrap_ui (UI_SRCS
    grab_window.ui
    )
    
add_executable (${PROJECT_NAME} ${SRCS} ${UI_SRCS} ${MOC_SRCS})
target_link_libraries (${PROJECT_NAME} ${PACKET} ${GEOMETRY} ${QT_LIBRARIES})
