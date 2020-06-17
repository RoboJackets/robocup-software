#======================================================================
# Preamble
#======================================================================
cmake_minimum_required(VERSION 3.16)
project(rj_msgs)

#======================================================================
# Find package
#======================================================================
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

#======================================================================
# Define Targets
#======================================================================
# This creates the target
# rj_geometry_msgs__rosidl_typesupport_cpp
rosidl_generate_interfaces(rj_msgs
    # Messages
    msg/FieldDimensions.msg
    msg/GameSettings.msg
    msg/RawProtobuf.msg
    msg/VisionPacket.msg
    # Services
    srv/SetFieldDimensions.srv
    srv/SetGameSettings.srv
    DEPENDENCIES std_msgs builtin_interfaces
    )
