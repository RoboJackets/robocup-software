# ======================================================================
# Add sources
# ======================================================================
target_sources(rj_param_utils PRIVATE param.cpp ros2_param_provider.cpp)

add_executable(gpp_test global_param_main.cpp)
ament_target_dependencies(gpp_test PUBLIC rclcpp)


install(TARGETS
  gpp_test
  DESTINATION lib/${PROJECT_NAME})
