cmake_minimum_required(VERSION 3.8)
project(rj_vision_receiver)

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
find_package(rj_msgs REQUIRED)
find_package(rj_common REQUIRED)
find_package(rj_param_utils REQUIRED)
find_package(rj_utils REQUIRED)
find_package(rj_constants REQUIRED)
find_package(rj_protos REQUIRED)
find_package(rj_config_client REQUIRED)

set(VISION_RECEIVER_DEPS
  rclcpp
  Boost
  rj_msgs
  rj_common
  rj_param_utils
  rj_utils
  rj_constants
  rj_protos
  rj_config_client
)

set(VISION_RECEIVER_LIBS
  Boost::boost
)

set(VISION_RECEIVER_SRC
  src/vision_receiver.cpp
)

add_executable(${PROJECT_NAME}_node
  ${VISION_RECEIVER_SRC}
  src/main.cpp
)

target_link_libraries(${PROJECT_NAME}_node PUBLIC ${VISION_RECEIVER_LIBS})

target_include_directories(${PROJECT_NAME}_node
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}_node PUBLIC ${VISION_RECEIVER_DEPS})

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
