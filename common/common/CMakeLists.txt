find_package(Qt5 COMPONENTS Widgets Xml Core OpenGL Network Svg REQUIRED)

# Library and headers
add_library(common field_dimensions.cpp utils.cpp)
target_include_directories(common PUBLIC ${COMMON_INCLUDE_DIR})
target_link_libraries(common PUBLIC Qt5::Core Qt5::Widgets geometry2d proto_messages)

# For conversions to and from ros msgs
# Lmao I can't specify rosidl_target_interfaces to be public
# so I gotta jank my way through this:
# function(rosidl_target_interfaces target interface_target typesupport_name)
set(target common)
set(interface_target rj_robocup_msgs)
set(typesupport_name rosidl_typesupport_cpp)

set(typesupport_target "${interface_target}__${typesupport_name}")
add_dependencies(${target} ${interface_target})
get_target_property(include_directories ${typesupport_target} INTERFACE_INCLUDE_DIRECTORIES)
target_include_directories(${target} PUBLIC ${include_directories})
target_link_libraries(${target} PUBLIC ${typesupport_target})
