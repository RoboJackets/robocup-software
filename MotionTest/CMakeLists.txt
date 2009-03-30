project (motionTest)
cmake_minimum_required(VERSION 2.6)
# set (CMAKE_VERBOSE_MAKEFILE on)

add_definitions(-O2 -Wall -g3)

set (USE_QT true)
set (USE_NETWORK true)
set (USE_GEOMETRY true)
set (USE_FRAMEWORK true)
include ("${PROJECT_SOURCE_DIR}/../common/CMakeLists.txt")

#modules
include_directories (${PROJECT_SOURCE_DIR}/..)
add_subdirectory (${PROJECT_SOURCE_DIR}/../motion ${PROJECT_BINARY_DIR}/motion)
add_subdirectory (${PROJECT_SOURCE_DIR}/../log ${PROJECT_BINARY_DIR}/log)
add_subdirectory (${PROJECT_SOURCE_DIR}/../modeling ${PROJECT_BINARY_DIR}/modeling)

include_directories(${PROJECT_BINARY_DIR})

qt4_wrap_cpp (MOC_SRCS
    MainWindow.hpp
    RobotPath.hpp
)

qt4_wrap_ui (UI_HDRS
    ui/motion.ui
)

set (SRCS
    main.cpp
    MainWindow.cpp
    RobotPath.cpp
    ../soccer/Processor.cpp
    ../soccer/InputHandler.cpp
    ../soccer/JoystickInput.cpp
    ../soccer/RefereeHandler.cpp
    ../soccer/rrt.cpp
)

qt4_automoc (${SRCS})
set (exec motionTest)
add_executable (${exec} ${SRCS} ${MOC_SRCS} ${UI_HDRS})

target_link_libraries(${exec} ${PACKET_LIBRARY})
target_link_libraries(${exec} ${GEOMETRY_LIBRARY})
target_link_libraries(${exec} ${NETWORK_LIBRARY})
target_link_libraries(${exec} ${FRAMEWORK_LIBRARY})

target_link_libraries(${exec} ${QT_LIBRARIES})

#module linking
target_link_libraries(${exec} log)
target_link_libraries(${exec} motion)
target_link_libraries(${exec} modeling)
