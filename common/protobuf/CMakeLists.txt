
# positon-independent-code flag
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")

file(GLOB PROTO_FILES "${CMAKE_CURRENT_SOURCE_DIR}/*.proto")
protobuf_generate_cpp(PROTO_SRC, PROTO_INCLUDE ${PROTO_FILES})

add_library(proto_messages STATIC ${PROTO_SRC} ${PROTO_INCLUDE})
target_link_libraries(proto_messages ${PROTOBUF_LIBRARIES})
