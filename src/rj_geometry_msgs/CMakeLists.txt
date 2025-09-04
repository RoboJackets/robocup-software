cmake_minimum_required(VERSION 3.8)
project(rj_geometry_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(
  ${PROJECT_NAME}
  msg/Circle.msg
  msg/Line.msg
  msg/Point.msg
  msg/Polygon.msg
  msg/Pose.msg
  msg/Rect.msg
  msg/Segment.msg
  msg/ShapeSet.msg
  msg/Twist.msg
  DEPENDENCIES
  std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
