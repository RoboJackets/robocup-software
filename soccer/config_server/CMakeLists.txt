add_executable(config_server main.cpp config_server.cpp)

ament_target_dependencies(config_server rclcpp rclcpp_components)
rosidl_target_interfaces(config_server rj_robocup_msgs "rosidl_typesupport_cpp")

install(TARGETS config_server
  DESTINATION lib/${PROJECT_NAME})
