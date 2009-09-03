project (VisionGen)
cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

add_definitions(-O2 -Wall -g3)

set (USE_NETWORK true)
set (USE_QT true)
set (USE_GEOMETRY true)
include ("${PROJECT_SOURCE_DIR}/../common/CMakeLists.txt")

include_directories(${PROJECT_BINARY_DIR})

set (SRCS
	main.cpp
)

add_executable (visionGen ${SRCS})

target_link_libraries(visionGen ${GEOMETRY_LIBRARY} ${NETWORK_LIBRARY})
target_link_libraries(visionGen ${QT_LIBRARIES})
