cmake_minimum_required(VERSION 3.0.0)

project("GT RoboJackets RoboCup")


# include cmake files in the 'cmake folder'
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake")

# put executables in the 'run' folder
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/run)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/run)

# Google test
enable_testing()
include(SetupGTest)

# C++ version
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

# Build in debug mode
# TODO: make this easier to specify when running `make` on the command-line
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")

# Turn some compiler warnings into errors
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Werror=return-type")

# Qt5
if(APPLE)
    # look for the homebrew-installed version of Qt5
    find_package(Qt5Widgets
        REQUIRED
        PATHS /usr/local/Cellar/qt5/*)
else()
    find_package(Qt5Widgets REQUIRED)
endif()
message(STATUS "Found Qt5: ${Qt5Widgets_DIR}")

# Google Protobuf
find_package(Protobuf REQUIRED)
include_directories(${PROTOBUF_INCLUDE_DIR})

# Python
if(APPLE)
    # Finding homebrew-installed python3 on OS X doesn't work with the standard
    # FindPythonLibs.cmake file, so we do a custom search for it here.

    # This block will either fail or set these three variables:
    # * PYTHON_INCLUDE_DIRS
    # * PYTHON_LINK_DIRS
    # * PYTHON_LIBRARIES

    # try to find python3-config
    find_program(PYTHON3_CONFIG
        NAMES python3-config)
    if(NOT PYTHON3_CONFIG)
        message(FATAL_ERROR "Could not find python3-config")
    endif()

    # use python3-config to find where python stuff is stored
    execute_process(COMMAND "${PYTHON3_CONFIG}" --prefix
                    OUTPUT_VARIABLE PYTHON3_PREFIX
                    OUTPUT_STRIP_TRAILING_WHITESPACE
                    ERROR_QUIET)
    set(PYTHON_INCLUDE_DIRS "${PYTHON3_PREFIX}/Headers")
    set(PYTHON_LINK_DIRS "${PYTHON3_PREFIX}/lib")

    # search for the python library
    find_library(PYTHON_LIBRARIES
        NAMES python3.4 python3.3 python3.2 python3
        PATHS ${PYTHON_LINK_DIRS})
    if(NOT PYTHON_LIBRARIES)
        message(FATAL_ERROR "Unable to find python 3 library")
    endif()
else()
    find_package(PythonInterp 3.2 REQUIRED)
    find_package(PythonLibs 3.2 REQUIRED)
endif()

# Eigen - used for linear algebra
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN_INCLUDE_DIR})

# libusb
find_package(libusb-1.0 REQUIRED)

# SDL
find_package(SDL REQUIRED)

# Several things depend on the headers in the 'common' directory
include_directories("${PROJECT_SOURCE_DIR}/common") # for headers in common/
include_directories("${PROJECT_BINARY_DIR}/common") # for generated protobuf headers

# setup ccache to speed up recompiles.  It's especially useful when switching back and forth
# between branches where you'd normally have to recompile things many times.
# see http://stackoverflow.com/questions/1815688/how-to-use-ccache-with-cmake
if(NOT STATIC_ANALYSIS)
    set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE ccache)
    set_property(GLOBAL PROPERTY RULE_LAUNCH_LINK ccache)
endif()

# run all the other CMakeLists files
add_subdirectory(common)
add_subdirectory(soccer)
add_subdirectory(simulator)
add_subdirectory(logging)
add_subdirectory(firmware)
