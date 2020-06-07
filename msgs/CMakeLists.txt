# ROS2 messages
rosidl_generate_interfaces(rj_robocup_msgs
  # Messages
  "msg/GameSettings.msg"
  "msg/FieldDimensions.msg"
  "msg/Line.msg"
  "msg/Point.msg"
  "msg/Rect.msg"
  "msg/Segment.msg"
  # Services
  "srv/SetGameSettings.srv"
  # Actions
  DEPENDENCIES std_msgs
  LIBRARY_NAME rj_robocup_msgs
  )
