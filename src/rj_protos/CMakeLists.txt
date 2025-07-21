# ======================================================================
# Preamble
# ======================================================================
cmake_minimum_required(VERSION 3.16)
project(rj_protos LANGUAGES CXX)

# ======================================================================
# Find package
# ======================================================================
find_package(Protobuf REQUIRED)

# ======================================================================
# Define Targets
# ======================================================================
add_library(rj_protos STATIC)

# positon-independent-code flag
target_compile_options(rj_protos PRIVATE "-fPIC")

# ======================================================================
# Set Sources
# ======================================================================
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

# ======================================================================
# Add sources
# ======================================================================
set(PROTO_INCLUDE_DIR ${CMAKE_CURRENT_BINARY_DIR}/include/)

protobuf_generate_cpp(RJ_PROTOS_SRC PROTO_HDRS ${PROTO_FILES})
# Copy the headers to PROTO_INCLUDE_DIR/rj_protos
include(CopyProtobufHeaders)
copy_protobuf_headers("${PROTO_INCLUDE_DIR}/rj_protos" RJ_PROTOS_HDRS ${PROTO_HDRS})

target_sources(rj_protos PRIVATE ${RJ_PROTOS_SRC} ${RJ_PROTOS_HDRS})

# ======================================================================
# Include and Linking
# ======================================================================
target_link_libraries(rj_protos ${PROTOBUF_LIBRARIES})
target_include_directories(rj_protos SYSTEM PUBLIC ${PROTO_INCLUDE_DIR})

# ======================================================================
# Packaging
# ======================================================================
install(DIRECTORY ${PROTO_INCLUDE_DIR} DESTINATION include/)
