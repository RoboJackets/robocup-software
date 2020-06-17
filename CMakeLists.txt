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

option(NO_WALL "Disables -Wall for CI so it doesn't print as much." OFF)

if(NOT NO_WALL AND CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
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
add_subdirectory(external)
add_subdirectory(rj_geometry_msgs)
add_subdirectory(rj_msgs)
add_subdirectory(rj_protos)
add_subdirectory(rj_constants)
add_subdirectory(Geometry2d) # TODO(1516)
add_subdirectory(rj_config)
add_subdirectory(rj_common)
add_subdirectory(soccer)

#======================================================================
# Packaging
#======================================================================
install(DIRECTORY launch
        DESTINATION share/${CMAKE_PROJECT_NAME})

install(FILES
        external/sdlcontrollerdb/gamecontrollerdb.txt
        DESTINATION share/${CMAKE_PROJECT_NAME})
