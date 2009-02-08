project (Soccer)
cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

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

set (SRCS
	main.cpp
	Processor.cpp
	MainWindow.cpp
	InputHandler.cpp
	JoystickInput.cpp
)

qt4_automoc (${SRCS})

set (target soccer)
add_executable (${target} ${SRCS} ${MOC_SRCS})

target_link_libraries(${target} ${PACKET_LIBRARY})
target_link_libraries(${target} ${GEOMETRY_LIBRARY})
target_link_libraries(${target} ${NETWORK_LIBRARY})
target_link_libraries(${target} ${FRAMEWORK_LIBRARY})

target_link_libraries(${target} ${QT_LIBRARIES})

#Module Linking
# Add a new line here for every module used
target_link_libraries(${target} log)
target_link_libraries(${target} motion)
target_link_libraries(${target} modeling)
