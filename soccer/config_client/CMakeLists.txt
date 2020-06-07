add_library(config_client STATIC config_client.cpp config_client_node.cpp)
rosidl_target_interfaces(config_client rj_robocup_msgs "rosidl_typesupport_cpp")
ament_target_dependencies(config_client rclcpp)

install(TARGETS
        config_client
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin)
