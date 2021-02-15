# ======================================================================
# Add
# ======================================================================
find_package(rj_msgs REQUIRED)
add_executable(gpp_test global_param_main.cpp)
set(GPP_TEST_INCLUDE 
    ../include
    ../../rj_constants/include
    ../../rj_vision_receiver/include)

set(GPP_TEST_LIBRARIES 
    rj_param_utils
    rj_constants
    rj_vision_receiver
    config_client)

# target_sources(rj_param_utils PRIVATE param.cpp ros2_param_provider.cpp)
target_include_directories(gpp_test PUBLIC ${GPP_TEST_INCLUDE})

# target_include_directories(gpp_test PUBLIC ../include/rj_param_utils)
target_link_libraries(gpp_test PUBLIC ${GPP_TEST_LIBRARIES})

ament_target_dependencies(gpp_test PUBLIC rclcpp)


install(
  TARGETS gpp_test
  DESTINATION lib/gpp_test)
