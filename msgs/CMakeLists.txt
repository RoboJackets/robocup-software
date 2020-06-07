find_package(builtin_interfaces REQUIRED)

# ROS2 messages
rosidl_generate_interfaces(rj_robocup_msgs
        # Messages
        #    Geometry2d
        "msg/Line.msg"
        "msg/Point.msg"
        "msg/Rect.msg"
        "msg/Segment.msg"
        #    RJ Robocup
        "msg/GameSettings.msg"
        "msg/FieldDimensions.msg"
        #    misc
        "msg/RawProtobuf.msg"
        "msg/VisionPacket.msg"

        # Services
        "srv/SetGameSettings.srv"
        "srv/SetFieldDimensions.srv"

        # Actions
        DEPENDENCIES std_msgs builtin_interfaces
        LIBRARY_NAME rj_robocup_msgs
        )
