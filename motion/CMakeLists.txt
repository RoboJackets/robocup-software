project (MotionModule)
cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

add_definitions(-O2 -Wall -g3)

set (USE_QT true)
set (USE_NETWORK true)
set (QT_USE_QTXML true)
set (USE_GEOMETRY true)
set (USE_FRAMEWORK true)
set (USE_CONFIG true)
include ("${PROJECT_SOURCE_DIR}/../common/CMakeLists.txt")


include_directories(${PROJECT_BINARY_DIR})

qt4_wrap_cpp ( MOC_SRCS

)

set (SRCS
    LinearController.cpp
    Controller.cpp
    Robot.cpp
    Pid.cpp
    main.cpp
)

add_library (motion ${SRCS} ${MOC_SRCS})
target_link_libraries (motion ${QT_LIBRARIES} ${GEOMETRY_LIBRARY} ${FRAMEWORK_LIBRARY} ${NETWORK_LIBRARY} ${CONFIG_LIBRARY})

