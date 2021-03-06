# ======================================================================
# Preamble
# ======================================================================
cmake_minimum_required(VERSION 3.16)
project(rj_robocup LANGUAGES CXX)

# ======================================================================
# C++ Version and Compiler Flags
# ======================================================================

set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ======================================================================
# CMake Options.
# ======================================================================
option(NO_WALL "Disables -Wall for CI so it doesn't print as much." OFF)
option(BUILD_DOCUMENTATION "Build documentation if doxygen and sphinx are found." OFF)

if(NOT NO_WALL
   AND CMAKE_COMPILER_IS_GNUCXX
   OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Set a default build type if none was specified
set(default_build_type "RelWithDebInfo")

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  message(STATUS "Setting build type to '${default_build_type}' as none was specified.")
  set(CMAKE_BUILD_TYPE
      "${default_build_type}"
      CACHE STRING "Choose the type of build." FORCE)
  # Set the possible values of build type for cmake-gui
  set_property(
    CACHE CMAKE_BUILD_TYPE
    PROPERTY STRINGS
             "Debug"
             "Release"
             "MinSizeRel"
             "RelWithDebInfo")
endif()

# ======================================================================
# CMake things
# ======================================================================

# A separate flag for now
option(BUILD_TESTING "Build tests." ON)

# include cmake files in the 'cmake folder'
list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")

# ======================================================================
# Compile Flags
# ======================================================================

# Because we use ninja, we have to explicitly turn on color output for the compiler
if("${CMAKE_CXX_COMPILER_ID}" MATCHES "Clang")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fcolor-diagnostics -Werror=return-stack-address")
else()
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fdiagnostics-color=always -Werror=return-local-addr")
endif()

# Use compiler optimization if we are making a release build.
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O2 -march=native")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g -Og")
set(CMAKE_CXX_FLAGS_DEBUG
    "${CMAKE_CXX_FLAGS_DEBUG} -Werror=return-type -Werror=delete-non-virtual-dtor")

# ======================================================================
# Testing
# ======================================================================

# Google test
enable_testing()
include(SetupGTest)

# ======================================================================
# CCache
# ======================================================================

find_program(CCACHE_PROGRAM ccache)
if(CCACHE_PROGRAM)
  set(CMAKE_CXX_COMPILER_LAUNCHER "${CCACHE_PROGRAM}")
endif()

# ======================================================================
# Subdirectories
# ======================================================================
include(AddROSIDLSubdirectory)

# First call add_idl_subdirectory on all ROSIDL subdirectories. This will build + install them at
# configure time so that find_package will work on them.
add_rosidl_subdirectory(rj_geometry_msgs)
add_rosidl_subdirectory(rj_drawing_msgs)
add_rosidl_subdirectory(rj_msgs)

# This is manually topologically sorted because we're not using colcon
add_subdirectory(rj_convert)
add_subdirectory(rj_param_utils)
add_subdirectory(rj_topic_utils)
add_subdirectory(external)
add_subdirectory(rj_protos)
add_subdirectory(rj_constants)
add_subdirectory(rj_utils)
add_subdirectory(rj_geometry)
add_subdirectory(rj_config)
add_subdirectory(rj_common)

add_subdirectory(rj_gameplay)
add_subdirectory(rj_vision_receiver)
add_subdirectory(soccer)

add_subdirectory(docs)

# ======================================================================
# Packaging
# ======================================================================
install(DIRECTORY launch DESTINATION share/${CMAKE_PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${CMAKE_PROJECT_NAME})

install(FILES external/sdlcontrollerdb/gamecontrollerdb.txt DESTINATION share/${CMAKE_PROJECT_NAME})

# ======================================================================
# Ament packaging
# ======================================================================
find_package(ament_cmake REQUIRED)
ament_package()
