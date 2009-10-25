cmake_minimum_required(VERSION 2.6)

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
			COMMAND protoc --cpp_out=${CMAKE_CURRENT_BINARY_DIR} --proto_path=${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/${proto_file}
			DEPENDS ${proto_file}
			VERBATIM
	)
	
	list (APPEND GEN_CC ${CMAKE_CURRENT_BINARY_DIR}/${cc_file})
	list (APPEND GEN_H ${CMAKE_CURRENT_BINARY_DIR}/${h_file})
endforeach()

add_custom_target(GenerateProto
	DEPENDS ${GEN_CC} ${GEN_H}
	VERBATIM
)

include_directories(${CMAKE_CURRENT_BINARY_DIR})

add_library(vision ${GEN_CC})
add_dependencies(vision GenerateProto)
target_link_libraries(vision protobuf)
