cmake_minimum_required(VERSION 3.8)
project(rj_protos LANGUAGES CXX)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Protobuf REQUIRED)

protobuf_generate_cpp(
  PROTOBUF_SRCS
  PROTOBUF_HDRS
  protos/Control.proto
  protos/grSim_Commands.proto
  protos/grSim_Packet.proto
  protos/grSim_Replacement.proto
  protos/LogFrame.proto
  protos/Point.proto
  protos/RadioRx.proto
  protos/RadioTx.proto
  protos/referee.proto
  protos/Robot.proto
  protos/ssl_gc_common.proto
  protos/ssl_gc_geometry.proto
  protos/ssl_gc_game_event.proto
  protos/ssl_simulation_config.proto
  protos/ssl_simulation_control.proto
  protos/ssl_simulation_error.proto
  protos/ssl_simulation_robot_control.proto
  protos/ssl_simulation_robot_feedback.proto
  protos/ssl_simulation_synchronous.proto
  protos/ssl_vision_detection.proto
  protos/ssl_vision_geometry.proto
  protos/ssl_vision_wrapper.proto
)

add_library(${PROJECT_NAME} SHARED
  ${PROTOBUF_HDRS}
  ${PROTOBUF_SRCS}
)

target_link_libraries(${PROJECT_NAME}
  protobuf::libprotobuf
)

target_include_directories(${PROJECT_NAME}
  INTERFACE
    $<BUILD_INTERFACE:${CMAKE_BINARY_DIR}>
    $<INSTALL_INTERFACE:include>
)

ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(Protobuf)

install(
  DIRECTORY ${CMAKE_BINARY_DIR}
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_export_targets(${PROJECT_NAME})
ament_package()
