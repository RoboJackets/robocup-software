cmake_minimum_required(VERSION 3.0.0)

project("GT RoboJackets RoboCup")


# include cmake files in the 'cmake folder'
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")


# Qt5
set(CMAKE_AUTOMOC ON)
find_package(Qt5Widgets)

# Google Protobuf
find_package(Protobuf REQUIRED)
include_directories(${PROTOBUF_INCLUDE_DIR})

# Eigen - used for linear algebra
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN_INCLUDE_DIR})

include_directories("${PROJECT_BINARY_DIR}/common")

add_subdirectory(common)
add_subdirectory(soccer)
add_subdirectory(logging)
add_subdirectory(firmware)
