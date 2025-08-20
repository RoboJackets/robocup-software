cmake_minimum_required(VERSION 3.8)
project(rj_drawing_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rj_geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(
  ${PROJECT_NAME}

  # Messages
  msg/DebugDraw.msg
  msg/DrawColor.msg
  msg/DrawShapes.msg
  msg/DrawSegment.msg
  msg/DrawPose.msg
  msg/DrawPath.msg
  msg/DrawText.msg

  DEPENDENCIES
  std_msgs
  builtin_interfaces
  rj_geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
