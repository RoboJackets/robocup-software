cmake_minimum_required(VERSION 3.8)
project(rj_referee)

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
find_package(Boost REQUIRED)
find_package(fmt REQUIRED)
find_package(spdlog REQUIRED)
find_package(rj_common REQUIRED)
find_package(rj_constants REQUIRED)
find_package(rj_param_utils REQUIRED)
find_package(rj_utils REQUIRED)
find_package(rj_msgs REQUIRED)
find_package(rj_protos REQUIRED)
find_package(rj_config_client REQUIRED)
find_package(rj_benchmarking REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/referee_base.cpp
  src/external_referee.cpp
  src/internal_referee.cpp
)

add_executable(external_referee_node
  src/external_referee.cpp
  src/external_referee_main.cpp
  src/referee_base.cpp
)

add_executable(internal_referee_node
  src/internal_referee.cpp
  src/internal_referee_main.cpp
  src/referee_base.cpp
)

target_link_libraries(${PROJECT_NAME}
  PUBLIC
  Boost::boost
  fmt
  spdlog
)

target_link_libraries(external_referee_node
  PUBLIC
  Boost::boost
  fmt
  spdlog
)

target_link_libraries(internal_referee_node
  PUBLIC
  Boost::boost
  fmt
  spdlog
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_include_directories(external_referee_node
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_include_directories(internal_referee_node
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  PUBLIC
  rclcpp
  rclcpp_components
  Boost
  fmt
  spdlog
  rj_common
  rj_constants
  rj_param_utils
  rj_utils
  rj_msgs
  rj_protos
  rj_config_client
  rj_benchmarking
)

ament_target_dependencies(external_referee_node
  PUBLIC
  rclcpp
  rclcpp_components
  Boost
  fmt
  spdlog
  rj_common
  rj_constants
  rj_param_utils
  rj_utils
  rj_msgs
  rj_protos
  rj_config_client
  rj_benchmarking
)

ament_target_dependencies(internal_referee_node
  PUBLIC
  rclcpp
  rclcpp_components
  Boost
  fmt
  spdlog
  rj_common
  rj_constants
  rj_param_utils
  rj_utils
  rj_msgs
  rj_protos
  rj_config_client
  rj_benchmarking
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS external_referee_node internal_referee_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_export_targets(${PROJECT_NAME})
ament_package()
