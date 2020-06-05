add_library(config_server SHARED config_server.cpp)

ament_target_dependencies(config_server rclcpp rclcpp_components)
rosidl_target_interfaces(config_server ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(config_server_standalone config_server_standalone.cpp)
target_link_libraries(config_server_standalone config_server)
ament_target_dependencies(config_server_standalone rclcpp)

install(TARGETS
  config_server
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS
  config_server_standalone
  DESTINATION lib/config_server_standalone)
