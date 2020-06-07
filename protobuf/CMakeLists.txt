project(rj_robocup_protobuf)

find_package(Protobuf REQUIRED)

# position-independent-code flag
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")

file(GLOB PROTO_FILES "${CMAKE_CURRENT_SOURCE_DIR}/*.proto")
protobuf_generate_cpp(PROTO_SRC PROTO_INCLUDE ${PROTO_FILES})

set(PROTO_INCLUDE_DIR ${CMAKE_CURRENT_BINARY_DIR}/include/)

add_library(proto_messages STATIC ${PROTO_SRC} ${PROTO_INCLUDE})
target_link_libraries(proto_messages ${PROTOBUF_LIBRARIES})
target_include_directories(proto_messages PUBLIC ${PROTO_INCLUDE_DIR})

message("\n\n proto include: ${PROTO_INCLUDE}")
message("binary dir: ${CMAKE_CURRENT_BINARY_DIR}")
message("proto include dir: ${PROTO_INCLUDE_DIR}")

# Copy the headers to an include directory so we can install it
file(MAKE_DIRECTORY ${PROTO_INCLUDE_DIR}/rj_robocup_protobuf)

add_custom_command(TARGET proto_messages
  POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E copy ${PROTO_INCLUDE} ${PROTO_INCLUDE_DIR}/rj_robocup_protobuf
  )

install(DIRECTORY ${PROTO_INCLUDE_DIR}
  DESTINATION include/
  )

