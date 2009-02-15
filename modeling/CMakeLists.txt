project (WorldModel)
cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

add_definitions(-O2 -Wall -g3)

set (USE_QT true)
set (USE_NETWORK true)
set (QT_USE_QTXML true)
set (USE_GEOMETRY true)
set (USE_FRAMEWORK true)
include ("${PROJECT_SOURCE_DIR}/../common/CMakeLists.txt")


include_directories(${PROJECT_BINARY_DIR})

qt4_wrap_cpp ( MOC_SRCS

)

set (SRCS
	WorldModel.cpp
	RobotWM.cpp
)

add_library (modeling ${SRCS} ${MOC_SRCS})
target_link_libraries (modeling ${QT_LIBRARIES} ${GEOMETRY_LIBRARY} ${FRAMEWORK_LIBRARY} ${NETWORK_LIBRARY})

