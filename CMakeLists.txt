cmake_minimum_required(VERSION 3.0.0)

project("GT RoboJackets RoboCup")


# include cmake files in the 'cmake folder'
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake")

# Compiler flags
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

# put executables in the 'run' folder
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/run)

# Qt5
set(CMAKE_AUTOMOC ON)
find_package(Qt5Widgets)

# Google Protobuf
find_package(Protobuf REQUIRED)
include_directories(${PROTOBUF_INCLUDE_DIR})

# Eigen - used for linear algebra
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN_INCLUDE_DIR})

# Several things depend on the headers in the 'common' directory
include_directories("${PROJECT_BINARY_DIR}/common")

# run all the other CMakeLists files
add_subdirectory(common)
add_subdirectory(soccer)
add_subdirectory(logging)
add_subdirectory(firmware)
