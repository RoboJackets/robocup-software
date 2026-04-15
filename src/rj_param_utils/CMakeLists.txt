cmake_minimum_required(VERSION 3.8)
project(rj_param_utils)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(fmt REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/global_params.cpp
  src/param.cpp
  src/ros2_global_param_provider.cpp
  src/ros2_local_param_provider.cpp
  src/ros2_param_provider.cpp
  src/planning/planning_params.cpp
  src/vision/vision_params.cpp
)
set_target_properties(${PROJECT_NAME} PROPERTIES UNITY_BUILD ON)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  fmt
)

target_link_libraries(${PROJECT_NAME}
  fmt
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

add_executable(global_param_server_node
  src/global_param_server.cpp
)

target_link_libraries(global_param_server_node PRIVATE ${PROJECT_NAME})

ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS global_param_server_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_targets(${PROJECT_NAME})
ament_package()
