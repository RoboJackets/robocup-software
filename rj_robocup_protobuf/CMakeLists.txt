cmake_minimum_required(VERSION 3.5)
project(rj_robocup_protobuf)

find_package(ament_cmake REQUIRED)
find_package(rj_robocup_cmake_utils)

# Google Protobuf
find_package(Protobuf REQUIRED)
include_directories(SYSTEM ${PROTOBUF_INCLUDE_DIR})

# Sets default build type to RELEASE, C++17, compiler flags
rj_robocup_package()

# positon-independent-code flag
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")

file(GLOB PROTO_FILES "${CMAKE_CURRENT_SOURCE_DIR}/protos/*.proto")
protobuf_generate_cpp(PROTO_SRC PROTO_INCLUDE ${PROTO_FILES})

add_library(proto_messages STATIC ${PROTO_SRC} ${PROTO_INCLUDE})
target_link_libraries(proto_messages ${PROTOBUF_LIBRARIES})

install(TARGETS
        proto_messages
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

ament_export_libraries(proto_messages)
ament_package()
