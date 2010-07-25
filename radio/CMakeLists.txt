project (Radio)
cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

add_definitions(-O2 -Wall -g3)

include ("${PROJECT_SOURCE_DIR}/../common/CMakeLists.txt")

include_directories(${PROJECT_BINARY_DIR})

set (SRCS
	main.cpp USB_Device.cpp Radio.cpp
)

set (exec radio)
add_executable (${exec} ${SRCS} ${MOC_SRCS})

target_link_libraries(${exec} packets)
target_link_libraries(${exec} ${GEOMETRY_LIBRARY})
target_link_libraries(${exec} ${QT_LIBRARIES})
target_link_libraries(${exec} usb)
