#======================================================================
# Preamble
#======================================================================
cmake_minimum_required(VERSION 3.16)
project(soccer LANGUAGES CXX)

#======================================================================
# Find package
#======================================================================
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)

# Qt5
find_package(Qt5 COMPONENTS Widgets Xml Core OpenGL Network Svg REQUIRED)
message(STATUS "Found Qt5: ${Qt5Widgets_DIR}")
add_definitions(-DQT_NO_KEYWORDS) # Remove QT slots/signals/emit keywords

# Google Protobuf
find_package(Protobuf REQUIRED)
include_directories(SYSTEM ${PROTOBUF_INCLUDE_DIR})

# Python
find_package(PythonInterp 3.2 REQUIRED)
find_package(PythonLibs 3.2 REQUIRED)

# Eigen - used for linear algebra
find_package(Eigen3 REQUIRED)

# libusb
find_package(libusb-1.0 REQUIRED)

# SDL2
include(FindPkgConfig)
pkg_search_module(SDL2 REQUIRED sdl2)

# Pthread
find_package(Threads REQUIRED)

# Boost Python
set(Boost_USE_STATIC_LIBS OFF)
set(Boost_USE_MULTITHREADED ON)
set(Boost_USE_STATIC_RUNTIME OFF)
find_package(Boost COMPONENTS system REQUIRED)

# find boost python
# This package is named 'python3' on Arch, 'python-py34' on Ubuntu 14, and 'python' on OS X
set(FOUND_A_BOOST_PYTHON FALSE)
# Find the matching boost python implementation
set(version ${PYTHONLIBS_VERSION_STRING})

STRING(REGEX REPLACE "[^0-9]" "" boost_py_version "${version}")
# Get first two numbers of python string
STRING(REGEX MATCH "^[0-9][0-9]" boost_py_version "${boost_py_version}")

foreach (possible_name python3 python-py${boost_py_version} python)
    find_package(Boost COMPONENTS ${possible_name} QUIET)
    if (Boost_FOUND)
        set(FOUND_A_BOOST_PYTHON TRUE)
        break()
    endif ()
endforeach ()
if (NOT FOUND_A_BOOST_PYTHON)
    message(FATAL_ERROR "Unable to find a suitable version of boost python")
endif ()

#======================================================================
# Define Targets
#======================================================================
# Build stand-alone soccer dylib
# This is linked into soccer and our unit tests, as well as being a python module
add_library(robocup SHARED)

set_target_properties(robocup PROPERTIES PREFIX "")
set_target_properties(robocup PROPERTIES SUFFIX ".so")

# Qt
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

add_executable(soccer)
#add_executable(log_viewer)

add_subdirectory(src)

#======================================================================
# Dependencies List
#======================================================================

# Robocup (static lib)
set(ROBOCUP_DEPS_SYSTEM_INCLUDE_DIRS
    ${EIGEN_INCLUDE_DIRS}
    ${LIBUSB_1_INCLUDE_DIRS}
    ${PYTHON_INCLUDE_DIRS}
    ${RRT_INCLUDE_DIR}
    ${SDL2_INCLUDE_DIRS})

set(ROBOCUP_DEPS_SYSTEM_LIBRARIES
    ${Boost_LIBRARIES}
    ${Boost_SYSTEM_LIBRARY}
    GL
    GLU
    glut
    ${PYTHON_LINK_DIRS}
    ${PYTHON_LIBRARIES}
    Qt5::Widgets
    Qt5::Xml
    Qt5::Core
    Qt5::OpenGL
    Qt5::Network
    Qt5::Svg
    Threads::Threads
    RRT
    spnav
    ${SDL2_LIBRARIES}
    )

set(ROBOCUP_DEPS_INCLUDE_DIRS
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    )

set(ROBOCUP_DEPS_LIBRARIES
    rj_common
    rj_protos
    rc-fshare
    )

# Soccer (executable)
set(SOCCER_DEPS_SYSTEM_LIBRARIES
    Qt5::Widgets
    Qt5::Xml
    Qt5::Core
    Qt5::OpenGL
    Qt5::Network
    Qt5::Svg)

set(SOCCER_DEPS_LIBRARIES
    robocup)

#======================================================================
# Include and Linking
#======================================================================

# ---- Robocup ----
ament_target_dependencies(robocup PUBLIC ament_index_cpp)

target_include_directories(robocup
    SYSTEM PUBLIC
    ${ROBOCUP_DEPS_SYSTEM_INCLUDE_DIRS})
target_include_directories(robocup
    PUBLIC
    ${ROBOCUP_DEPS_INCLUDE_DIRS})
target_link_libraries(robocup
    PUBLIC
    ${ROBOCUP_DEPS_SYSTEM_LIBRARIES}
    ${ROBOCUP_DEPS_LIBRARIES})

target_link_libraries(rj_common
    PUBLIC
    ${RJ_COMMON_DEPS_SYSTEM_LIBRARIES}
    ${RJ_COMMON_DEPS_LIBRARIES})

# ---- Soccer ----
target_link_libraries(soccer
    PUBLIC
    ${SOCCER_DEPS_SYSTEM_LIBRARIES}
    ${SOCCER_DEPS_LIBRARIES})

## build the 'log_viewer' program
#target_link_libraries(log_viewer Core Widgets OpenGL Svg Xml)
#target_link_libraries(log_viewer robocup)

#======================================================================
# Testing
#======================================================================

## Add a test runner target "test-soccer" to run all tests in this directory
#if (BUILD_TESTS)
#    add_executable(test-soccer ${SOCCER_TEST_SRC})
#    target_link_libraries(test-soccer robocup)
#    qt5_use_modules(test-soccer Core Widgets Xml)
#    target_link_libraries(test-soccer ${GTEST_LIBRARIES})
#    add_dependencies(test-soccer googletest)
#endif ()

#======================================================================
# Packaging
#======================================================================
