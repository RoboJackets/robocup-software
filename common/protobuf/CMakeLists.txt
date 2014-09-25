
# positon-independent-code flag
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")

file(GLOB ProtoFiles "${CMAKE_CURRENT_SOURCE_DIR}/*.proto")
protobuf_generate_cpp(ProtoSources, ProtoHeaders ${ProtoFiles})
add_library(proto_messages STATIC ${ProtoSources} ${ProtoHeaders})
