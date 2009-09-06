project (MotionModule)
cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

add_definitions(-O2 -Wall -g3)

set (USE_QT true)
set (USE_NETWORK true)
set (QT_USE_QTXML true)
set (USE_GEOMETRY true)
set (USE_FRAMEWORK true)
include ("${PROJECT_SOURCE_DIR}/../common/CMakeLists.txt")

include_directories(${PROJECT_SOURCE_DIR}/../)
include_directories(${PROJECT_BINARY_DIR})

qt4_wrap_cpp ( MOC_SRCS
    MotionModule.hpp
)

qt4_wrap_ui (UI_HDRS
    ui/config.ui
)

set (SRCS
    MotionModule.cpp
    Robot.cpp
	Pid.cpp
    Dynamics.cpp
	
    planning/rrt.cpp
    planning/Tree.cpp
)

set (target motion)
add_library (${target} ${SRCS} ${MOC_SRCS} ${UI_HDRS})

target_link_libraries (${target} ${QT_LIBRARIES} )
target_link_libraries (${target} ${GEOMETRY_LIBRARY})
target_link_libraries (${target} ${FRAMEWORK_LIBRARY})
target_link_libraries (${target} ${NETWORK_LIBRARY})
target_link_libraries (${target} ${CONFIG_LIBRARY})
