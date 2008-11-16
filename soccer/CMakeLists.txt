project (TestFramework)
cmake_minimum_required(VERSION 2.6)

add_definitions(-O2 -Wall -g3)

set (USE_QT true)
set (USE_NETWORK true)
set (USE_GEOMETRY true)
set (USE_FRAMEWORK true)
include ("${PROJECT_SOURCE_DIR}/../common/CMakeLists.txt")

add_subdirectory(${PROJECT_SOURCE_DIR}/../log ${PROJECT_BINARY_DIR}/log)
include_directories (${PROJECT_SOURCE_DIR}/../log)

include_directories(${PROJECT_BINARY_DIR})

qt4_wrap_cpp ( MOC_SRCS
	MainWindow.hpp
)

set (SRCS
	main.cpp
	TeamHandler.cpp
	MainWindow.cpp
	
	module/WorldModel.cpp
	module/LogModule.cpp
)

add_executable (testFramework ${SRCS} ${MOC_SRCS})

target_link_libraries(testFramework ${PACKET_LIBRARY})
target_link_libraries(testFramework ${GEOMETRY_LIBRARY})
target_link_libraries(testFramework ${NETWORK_LIBRARY})
target_link_libraries(testFramework ${FRAMEWORK_LIBRARY})

target_link_libraries(testFramework ${QT_LIBRARIES})
target_link_libraries(testFramework log)
