#======================================================================
# Preamble
#======================================================================
cmake_minimum_required(VERSION 3.16)
project(rj_robocup LANGUAGES CXX)

#======================================================================
# C++ Version and Compiler Flags
#======================================================================

set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

#======================================================================
# CMake things
#======================================================================

# A separate flag for now
option(BUILD_TESTING "Build tests." ON)

# include cmake files in the 'cmake folder'
list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")

#======================================================================
# Compile Flags
#======================================================================

# Because we use ninja, we have to explicitly turn on color output for the compiler
if ("${CMAKE_CXX_COMPILER_ID}" MATCHES "Clang")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fcolor-diagnostics -Werror=return-stack-address")
else()
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Werror=return-local-addr")
endif()

# Use compiler optimization if we are making a release build.
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O2 -march=native")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g -Og")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Werror=return-type -Werror=delete-non-virtual-dtor")

#======================================================================
# Testing
#======================================================================

# Google test
enable_testing()
include(SetupGTest)

#======================================================================
# CCache
#======================================================================

find_program(CCACHE_PROGRAM ccache)
if(CCACHE_PROGRAM)
    set(CMAKE_CXX_COMPILER_LAUNCHER "${CCACHE_PROGRAM}")
endif()

#======================================================================
# Subdirectories
#======================================================================
# This is manually topologically sorted becuse we're not using colcon
add_subdirectory(protobuf)
add_subdirectory(external)
add_subdirectory(common)
add_subdirectory(soccer)
#
#
## Qt5
#find_package(Qt5 COMPONENTS Widgets Xml Core OpenGL Network Svg REQUIRED)
#message(STATUS "Found Qt5: ${Qt5Widgets_DIR}")
#
## Google Protobuf
#find_package(Protobuf REQUIRED)
#include_directories(SYSTEM ${PROTOBUF_INCLUDE_DIR})
#
## Remove QT slots/signals/emit keywords
#add_definitions(-DQT_NO_KEYWORDS)
## Python
#find_package(PythonInterp 3.2 REQUIRED)
#find_package(PythonLibs 3.2 REQUIRED)
#
#
## Eigen - used for linear algebra
#find_package(Eigen3 REQUIRED)
#
## libusb
#find_package(libusb-1.0 REQUIRED)
#
## SDL2
#include(FindPkgConfig)
#pkg_search_module(SDL2 REQUIRED sdl2)
#
## Several things depend on the headers in the 'common' directory
#include_directories("${PROJECT_SOURCE_DIR}/common") # for headers in common/
#include_directories("${PROJECT_BINARY_DIR}/common") # for generated protobuf headers
#include_directories("${CMAKE_BINARY_DIR}/common") # for generated protobuf headers


#
## run all the other CMakeLists files
#
## Installing Targets
#install(TARGETS robocup
#        ARCHIVE DESTINATION lib
#        LIBRARY DESTINATION lib
#        RUNTIME DESTINATION bin)
#
#install(TARGETS grSim soccer
#        DESTINATION lib/${PROJECT_NAME})
#
#if(BUILD_TESTS)
#    install(TARGETS test-soccer
#            DESTINATION lib/${PROJECT_NAME})
#endif()
#
#install(DIRECTORY launch
#        DESTINATION share/${PROJECT_NAME})
#
#install(DIRECTORY
#        soccer/gameplay
#        DESTINATION share/${PROJECT_NAME}
#        FILES_MATCHING
#        PATTERN "*.py"
#        PATTERN "*.pbk")
#
#install(FILES
#        soccer/config/realShopField.xml
#        DESTINATION share/${PROJECT_NAME}/config
#        RENAME soccer-real.cfg)
#
#install(FILES
#        soccer/config/sim.xml
#        DESTINATION share/${PROJECT_NAME}/config
#        RENAME soccer-sim.cfg)
#
#install(FILES
#        external/sdlcontrollerdb/gamecontrollerdb.txt
#        DESTINATION share/${PROJECT_NAME})
#
#ament_package()
