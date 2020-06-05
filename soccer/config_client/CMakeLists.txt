add_library(config_client STATIC config_client.cpp)
rosidl_target_interfaces(config_client ${PROJECT_NAME} "rosidl_typesupport_cpp")
ament_target_dependencies(config_client rclcpp)

install(TARGETS
        config_client
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin)
