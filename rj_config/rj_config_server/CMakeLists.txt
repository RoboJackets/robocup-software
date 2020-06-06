cmake_minimum_required(VERSION 3.5)
project(rj_config_server)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rj_robocup_cmake_utils REQUIRED)
find_package(rj_robocup_msgs REQUIRED)

# Sets default build type to RELEASE, C++17, compiler flags
rj_robocup_package()
set(CMAKE_CXX_STANDARD 14)

add_executable(rj_config_server src/main.cpp src/config_server.cpp)
ament_target_dependencies(rj_config_server rclcpp rj_robocup_msgs)

install(TARGETS rj_config_server
        RUNTIME DESTINATION lib/${PROJECT_NAME}
        )

