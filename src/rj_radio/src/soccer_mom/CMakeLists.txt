add_executable(soccer_mom_node soccer_mom.cpp)

target_include_directories(
    soccer_mom_node
    PRIVATE
    ${PROJECT_SOURCE_DIR}/include
)

ament_target_dependencies(
    soccer_mom_node
    rclcpp
    std_msgs
    rj_msgs
)

install(
  TARGETS soccer_mom_node
  DESTINATION lib/${PROJECT_NAME}
)