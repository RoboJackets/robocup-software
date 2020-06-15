#======================================================================
# Preamble
#======================================================================
cmake_minimum_required(VERSION 3.16)
project(rj_protos LANGUAGES CXX)

#======================================================================
# Find package
#======================================================================
find_package(Protobuf REQUIRED)
find_package(ament_cmake REQUIRED)

#======================================================================
# Define Targets
#======================================================================
add_library(rj_protos STATIC)

# positon-independent-code flag
target_compile_options(rj_protos PRIVATE "-fPIC")

#======================================================================
# Set Sources
#======================================================================
set(PROTO_FILES
    protos/Control.proto
    protos/LogFrame.proto
    protos/Point.proto
    protos/RadioRx.proto
    protos/RadioTx.proto
    protos/Robot.proto
    protos/grSim_Commands.proto
    protos/grSim_Packet.proto
    protos/grSim_Replacement.proto
    protos/messages_robocup_ssl_detection.proto
    protos/messages_robocup_ssl_geometry.proto
    protos/messages_robocup_ssl_refbox_log.proto
    protos/messages_robocup_ssl_robot_status.proto
    protos/messages_robocup_ssl_wrapper.proto
    protos/referee.proto
    )

#======================================================================
# Add sources
#======================================================================
protobuf_generate_cpp(PROTO_SRC PROTO_INCLUDE ${PROTO_FILES})

target_sources(rj_protos
    PRIVATE
    ${PROTO_SRC})

#======================================================================
# Include and Linking
#======================================================================
set(PROTO_INCLUDE_DIR ${CMAKE_CURRENT_BINARY_DIR}/include/)

target_link_libraries(rj_protos ${PROTOBUF_LIBRARIES})
target_include_directories(rj_protos PUBLIC ${PROTO_INCLUDE_DIR})

#======================================================================
# Packaging
#======================================================================
# Copy the protos to include/rj_protos
file(MAKE_DIRECTORY ${PROTO_INCLUDE_DIR}/rj_protos)
add_custom_command(TARGET rj_protos
    PRE_LINK
    COMMAND ${CMAKE_COMMAND} -E copy ${PROTO_INCLUDE} ${PROTO_INCLUDE_DIR}/rj_protos
    )

install(DIRECTORY ${PROTO_INCLUDE_DIR}
    DESTINATION include/
    )

#======================================================================
# Ament packaging
#======================================================================
ament_export_libraries(rj_protos)
ament_package()
