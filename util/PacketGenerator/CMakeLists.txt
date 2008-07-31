## Build file for the packet generator ##

project (packetGenerator)

cmake_minimum_required(VERSION 2.6)

add_definitions (-Wall -Werror)

find_package (Qt4 REQUIRED)

#enable XML
set (QT_USE_QTXML true)

#disable qt gui
set (QT_DONT_USE_QTGUI true)

include (${QT_USE_FILE})

set (SRCS
	main.cpp
	Packet.cpp
	)

add_executable (${PROJECT_NAME} ${SRCS})

target_link_libraries(${PROJECT_NAME} ${QT_LIBRARIES})

# run the packet generator on all the packets to verify functionality
add_custom_target(test 
	COMMAND ${PROJECT_BINARY_DIR}/${PROJECT_NAME} ../../../common/Packet/packets/
	WORKING_DIRECTORY ${PROJECT_BINARY_DIR}
	VERBATIM
)
add_dependencies(test ${PROJECT_NAME})
