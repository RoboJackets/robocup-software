cmake_minimum_required(VERSION 3.5)
project(rj_robocup_bringup)

find_package(ament_cmake REQUIRED)
find_package(rj_robocup REQUIRED)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
