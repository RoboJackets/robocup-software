add_executable(vision_receiver vision_receiver.cpp)
ament_target_dependencies(vision_receiver rclcpp)
target_link_libraries(vision_receiver config_client common ${Boost_LIBRARIES} ${Boost_SYSTEM_LIBRARY})

install(TARGETS
        vision_receiver
        DESTINATION lib/vision_receiver)
