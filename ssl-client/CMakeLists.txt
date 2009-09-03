#ssl vision protoc -> robojackets packets converter
project (ssl-client)
cmake_minimum_required(VERSION 2.6)

set (USE_QT true)
set (USE_NETWORK true)
set (USE_GEOMETRY true)
set (USE_FRAMEWORK true)

include ("${PROJECT_SOURCE_DIR}/../common/CMakeLists.txt")

set (PROTO_FILES
	messages_robocup_ssl_detection
	messages_robocup_ssl_geometry
	messages_robocup_ssl_wrapper
)

set (GEN_CC)
set (GEN_H)

foreach (file ${PROTO_FILES})
	set (cc_file ${file}.pb.cc)
	set (h_file ${file}.pb.h)
	set (proto_file ${file}.proto)
	
	add_custom_command(OUTPUT ${cc_file} ${h_file}
			COMMAND protoc --cpp_out=${PROJECT_BINARY_DIR} --proto_path=${PROJECT_SOURCE_DIR} ${PROJECT_SOURCE_DIR}/${proto_file}
			DEPENDS ${proto_file}
			VERBATIM
	)
	
	list (APPEND GEN_CC ${PROJECT_BINARY_DIR}/${cc_file})
	list (APPEND GEN_H ${PROJECT_BINARY_DIR}/${h_file})
endforeach()

add_custom_target(GenerateProto
	    DEPENDS ${GEN_CC} ${GEN_H}
	    VERBATIM
	)

include_directories(${PROJECT_BINARY_DIR})

set (target ssl-client)
add_executable (${target} main.cpp ${GEN_CC})
add_dependencies(${target} GenerateProto)

target_link_libraries(${target} protobuf)
target_link_libraries(${target} ${PACKET_LIBRARY})
target_link_libraries(${target} ${FRAMEWORK_LIBRARY})
target_link_libraries(${target} ${GEOMETRY_LIBRARY})
target_link_libraries(${target} ${NETWORK_LIBRARY})
target_link_libraries(${target} ${QT_LIBRARIES})

