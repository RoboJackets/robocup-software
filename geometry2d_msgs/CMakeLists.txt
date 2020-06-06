cmake_minimum_required(VERSION 3.5)
project(geometry2d_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rj_robocup_cmake_utils REQUIRED)
find_package(std_msgs REQUIRED)

# Sets default build type to RELEASE, C++17, compiler flags
rj_robocup_package()

rosidl_generate_interfaces(${PROJECT_NAME}
        "msg/Line.msg"
        "msg/Point.msg"
        "msg/Rect.msg"
        "msg/Segment.msg"
        DEPENDENCIES std_msgs
        )

ament_export_dependencies(rosidl_default_runtime)

ament_package()

