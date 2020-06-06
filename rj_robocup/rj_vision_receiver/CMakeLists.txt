cmake_minimum_required(VERSION 3.5)
project(rj_vision_receiver)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rj_robocup_cmake_utils REQUIRED)
find_package(rj_robocup_msgs REQUIRED)
find_package(rj_config_client REQUIRED)
find_package(rj_robocup_protobuf)

find_package(Boost COMPONENTS system REQUIRED)

# Sets default build type to RELEASE, C++17, compiler flags
rj_robocup_package()

add_executable(vision_receiver src/vision_receiver.cpp)
ament_target_dependencies(vision_receiver rclcpp rj_config_client rj_robocup_protobuf)
target_link_libraries(vision_receiver ${Boost_LIBRARIES} ${Boost_SYSTEM_LIBRARY})

install(TARGETS vision_receiver
        RUNTIME DESTINATION lib/${PROJECT_NAME}
        )

