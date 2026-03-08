cmake_minimum_required(VERSION 3.8)
project(rj_config_client)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rj_msgs REQUIRED)
find_package(fmt REQUIRED)
find_package(rj_constants REQUIRED)
find_package(rj_common REQUIRED)
find_package(rj_utils REQUIRED)

set(RJ_CONFIG_CLIENT_DEPS
  rclcpp
  rj_msgs
  fmt
  rj_constants
  rj_common
  rj_utils
)

add_library(${PROJECT_NAME} SHARED
  src/config_client.cpp
  src/config_client_node.cpp
)

add_executable(${PROJECT_NAME}_node
  src/config_client.cpp
  src/config_client_node.cpp
)

target_link_libraries(${PROJECT_NAME}
  PUBLIC
  fmt
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_include_directories(${PROJECT_NAME}_node
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  PUBLIC
  ${RJ_CONFIG_CLIENT_DEPS}
)

ament_target_dependencies(${PROJECT_NAME}_node
  PUBLIC
  ${RJ_CONFIG_CLIENT_DEPS}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_targets(${PROJECT_NAME})
ament_package()
