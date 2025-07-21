cmake_minimum_required(VERSION 3.16)
project(external LANGUAGES C CXX)

find_package(ament_cmake REQUIRED)

add_subdirectory(robocup-fshare)
add_subdirectory(rrt)
add_subdirectory(googletest)

ament_package()