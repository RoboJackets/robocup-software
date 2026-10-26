cmake_minimum_required(VERSION 3.8)
project(rj_ui_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rj_geometry_msgs REQUIRED)

rosidl_generate_interfaces(
  ${PROJECT_NAME}
  msg/BallPlacement.msg
  msg/OverridePosition.msg
  msg/RobotPlacement.msg
  srv/SimPlacement.srv
  DEPENDENCIES
  rj_geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
