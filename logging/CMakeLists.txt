project (Soccer)
cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

add_definitions(-O2 -Wall -g3)

include (${PROJECT_SOURCE_DIR}/../common/CMakeLists.txt)

# Simple logger
add_executable(simple_logger
	simple_logger.cpp
	../common/multicast.cpp
	../common/Utils.cpp
)
target_link_libraries(simple_logger ${QT_LIBRARIES} packets)

# tcpdump log converter
add_executable(convert_tcpdump
	convert_tcpdump.cpp
)
target_link_libraries(convert_tcpdump packets pthread pcap)
