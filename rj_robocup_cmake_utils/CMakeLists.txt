cmake_minimum_required(VERSION 3.5)

project(rj_robocup_cmake_utils NONE)

find_package(ament_cmake_core REQUIRED)

ament_package(
        CONFIG_EXTRAS "rj_robocup_cmake_utils-extras.cmake"
)

message(STATUS "CMAKE_INSTALL_PREFIX: ${CMAKE_INSTALL_PREFIX}")

install(
        DIRECTORY cmake
        DESTINATION share/${PROJECT_NAME}
)
