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
# QT build tool things
#======================================================================
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

#======================================================================
# Define Targets
#======================================================================
# Build stand-alone soccer dylib
# This is linked into soccer and our unit tests, as well as being a python module
add_library(robocup SHARED)

set_target_properties(robocup PROPERTIES PREFIX "")
set_target_properties(robocup PROPERTIES SUFFIX ".so")

add_executable(soccer)
add_executable(log_viewer)

if (BUILD_TESTING)
    add_executable(test-soccer)
endif()

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
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_BINARY_DIR}/robocup_autogen/include
    )

set(ROBOCUP_DEPS_LIBRARIES
    rj_common
    rj_protos
    rc-fshare
    config_client
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

# log_viewer
set(LOG_VIEWER_DEPS_SYSTEM_LIBRARIES
    Qt5::Core
    Qt5::Widgets
    Qt5::OpenGL
    Qt5::Xml
    Qt5::Svg)

set(LOG_VIEWER_DEPS_LIBRARIES
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

ament_target_dependencies(robocup PUBLIC ament_index_cpp)

# ---- Soccer ----
target_link_libraries(soccer
    PUBLIC
    ${SOCCER_DEPS_SYSTEM_LIBRARIES}
    ${SOCCER_DEPS_LIBRARIES})

# ---- log_viewer ----
target_link_libraries(log_viewer
    PRIVATE
    ${LOG_VIEWER_DEPS_SYSTEM_LIBRARIES}
    ${LOG_VIEWER_DEPS_LIBRARIES})

## Add a test runner target "test-soccer" to run all tests in this directory
if (BUILD_TESTING)
    target_link_libraries(test-soccer PRIVATE robocup)
    target_link_libraries(test-soccer PRIVATE Qt5::Core Qt5::Widgets Qt5::Xml)

    set(test_libs geometry2d_testing rj_param_utils_testing)
    foreach(test_lib ${test_libs})
        target_link_libraries(test-soccer PRIVATE
            "-Wl,--whole-archive -L$<TARGET_FILE_DIR:${test_lib}> -l${test_lib} -Wl,--no-whole-archive")
        target_link_libraries(test-soccer PRIVATE ${test_lib})
    endforeach()

    target_link_libraries(test-soccer PRIVATE ${GTEST_LIBRARIES})
    add_dependencies(test-soccer googletest)
endif ()

#======================================================================
# Testing
#======================================================================
if(BUILD_TESTING)
    enable_testing()
    add_subdirectory(testing)
endif()

#======================================================================
# Packaging
#======================================================================
install(TARGETS robocup log_viewer
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin)

install(TARGETS soccer
        DESTINATION lib/${CMAKE_PROJECT_NAME})

install(DIRECTORY
        src/gameplay
        DESTINATION share/${CMAKE_PROJECT_NAME}
        FILES_MATCHING
        PATTERN "*.py"
        PATTERN "*.pbk")

install(FILES
        config/realShopField.xml
        DESTINATION share/${CMAKE_PROJECT_NAME}/config
        RENAME soccer-real.cfg)

install(FILES
        config/sim.xml
        DESTINATION share/${CMAKE_PROJECT_NAME}/config
        RENAME soccer-sim.cfg)

if(BUILD_TESTING)
    install(TARGETS test-soccer
            DESTINATION lib/${CMAKE_PROJECT_NAME})
endif()
