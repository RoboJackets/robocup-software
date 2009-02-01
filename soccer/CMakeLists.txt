project (TestFramework)
cmake_minimum_required(VERSION 2.6)

add_definitions(-O2 -Wall -g3)

set (USE_QT true)
set (USE_NETWORK true)
set (USE_GEOMETRY true)
set (USE_FRAMEWORK true)
include ("${PROJECT_SOURCE_DIR}/../common/CMakeLists.txt")

#Module Include Directories
# Add a new line here for every module used
include_directories (${PROJECT_SOURCE_DIR}/..)
add_subdirectory (${PROJECT_SOURCE_DIR}/../motion ${PROJECT_BINARY_DIR}/motion)
add_subdirectory (${PROJECT_SOURCE_DIR}/../log ${PROJECT_BINARY_DIR}/log)
add_subdirectory (${PROJECT_SOURCE_DIR}/../modeling ${PROJECT_BINARY_DIR}/modeling)

include_directories(${PROJECT_BINARY_DIR})

qt4_wrap_cpp ( MOC_SRCS
	MainWindow.hpp
)

set (SRCS
	main.cpp
	Processor.cpp
	MainWindow.cpp
)

set (exec soccer)
add_executable (${exec} ${SRCS} ${MOC_SRCS})

target_link_libraries(${exec} ${PACKET_LIBRARY})
target_link_libraries(${exec} ${GEOMETRY_LIBRARY})
target_link_libraries(${exec} ${NETWORK_LIBRARY})
target_link_libraries(${exec} ${FRAMEWORK_LIBRARY})

target_link_libraries(${exec} ${QT_LIBRARIES})

#Module Linking
# Add a new line here for every module used
target_link_libraries(${exec} log)
target_link_libraries(${exec} motion)
target_link_libraries(${exec} modeling)
