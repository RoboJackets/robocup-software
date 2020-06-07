find_package(rclcpp REQUIRED)

add_executable(vision_receiver main.cpp vision_receiver.cpp)
ament_target_dependencies(vision_receiver rclcpp)
target_link_libraries(vision_receiver
  config_client
  common
  proto_messages
  rj_robocup_network
  ${Boost_LIBRARIES}
  ${Boost_SYSTEM_LIBRARY})

install(TARGETS
        vision_receiver
        DESTINATION lib/${PROJECT_NAME})
