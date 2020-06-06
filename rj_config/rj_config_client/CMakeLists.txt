cmake_minimum_required(VERSION 3.5)
project(rj_config_client)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rj_robocup_cmake_utils REQUIRED)
find_package(rj_robocup_msgs REQUIRED)

# Sets default build type to RELEASE, C++17, compiler flags
rj_robocup_package()

add_library(rj_config_client STATIC src/config_client.cpp)
target_include_directories(rj_config_client PUBLIC include)
ament_target_dependencies(rj_config_client rclcpp rj_robocup_msgs)

install(TARGETS
        rj_config_client
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
        )

install(DIRECTORY include/
        DESTINATION include/
        )

ament_export_include_directories(include)
ament_export_libraries(rj_config_client)
ament_package()
