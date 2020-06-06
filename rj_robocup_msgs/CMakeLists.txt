cmake_minimum_required(VERSION 3.5)
project(rj_robocup_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rj_robocup_cmake_utils REQUIRED)
find_package(geometry2d_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Sets default build type to RELEASE, C++17, compiler flags
rj_robocup_package()

rosidl_generate_interfaces(${PROJECT_NAME}
        # Messages
        "msg/FieldDimensions.msg"
        "msg/GameSettings.msg"
        # Services
        "srv/SetFieldDimensions.srv"
        "srv/SetGameSettings.srv"
        # Actions
        DEPENDENCIES std_msgs geometry2d_msgs
        )

ament_export_dependencies(rosidl_default_runtime)

ament_package()

