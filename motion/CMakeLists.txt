project (MotionModule)
cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

add_definitions(-O2 -Wall -g3)

set (USE_QT true)
set (USE_NETWORK true)
include ("${PROJECT_SOURCE_DIR}/../common/CMakeLists.txt")

include_directories(${PROJECT_BINARY_DIR})

qt4_wrap_cpp ( MOC_SRCS
	
)

set (SRCS
	Controller.cpp
)

add_library (motion ${SRCS} ${MOC_SRCS})

#add_executable (test EXCLUDE_FROM_ALL test/test.cpp)
#target_link_libraries(test log ${QT_LIBRARIES})

