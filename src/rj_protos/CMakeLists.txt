# how to write/edit this file: https://docs.ros.org/en/humble/How-To-Guides/Ament-CMake-Documentation.html

cmake_minimum_required(VERSION 3.16)
project(rj_protos LANGUAGES CXX)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Enable useful warnings for GCC/Clang
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find required dependencies
find_package(ament_cmake REQUIRED)
find_package(Protobuf REQUIRED)


set(PROTO_FILES
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
    protos/ssl_vision_wrapper.proto)

# Generate .pb.cc and .pb.h from .proto files
protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS ${PROTO_FILES})

# Create the protobuf library
add_library(${PROJECT_NAME} STATIC
  ${PROTO_SRCS}
  ${PROTO_HDRS}
)

target_compile_options(${PROJECT_NAME} PRIVATE "-fPIC")

# Set include paths
target_include_directories(${PROJECT_NAME}
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)

# Link with protobuf
target_link_libraries(${PROJECT_NAME}
  PUBLIC ${Protobuf_LIBRARIES}
)

ament_target_dependencies(${PROJECT_NAME} PUBLIC Protobuf)

# Install the library
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install generated headers
install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.pb.h"
)

# Export targets for downstream CMake packages
install(EXPORT export_${PROJECT_NAME}
  FILE ${PROJECT_NAME}Targets.cmake
  NAMESPACE ${PROJECT_NAME}::
  DESTINATION share/${PROJECT_NAME}/cmake
)

ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(Protobuf)

ament_package()
