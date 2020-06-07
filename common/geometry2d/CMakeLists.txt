find_package(Eigen3 REQUIRED)
find_package(Qt5 COMPONENTS Widgets Xml Core OpenGL Network Svg REQUIRED)

# Library and headers
add_library(geometry2d
  src/arc.cpp
  src/circle.cpp
  src/line.cpp
  src/rect.cpp
  src/transform_matrix.cpp
  src/composite_shape.cpp
  src/point.cpp
  src/polygon.cpp
  src/segment.cpp
)
target_include_directories(geometry2d PUBLIC ${COMMON_INCLUDE_DIR} ${EIGEN_INCLUDE_DIR})
target_link_libraries(geometry2d PUBLIC Qt5::Core proto_messages)

# For conversions to and from ros msgs
# Lmao I can't specify rosidl_target_interfaces to be public
# so I gotta jank my way through this:
# function(rosidl_target_interfaces target interface_target typesupport_name)
set(target geometry2d)
set(interface_target rj_robocup_msgs)
set(typesupport_name rosidl_typesupport_cpp)

set(typesupport_target "${interface_target}__${typesupport_name}")
add_dependencies(${target} ${interface_target})
get_target_property(include_directories ${typesupport_target} INTERFACE_INCLUDE_DIRECTORIES)
target_include_directories(${target} PUBLIC ${include_directories})
target_link_libraries(${target} PUBLIC ${typesupport_target})
