project (LogModule)
cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

add_definitions(-O2 -Wall -g3)

set (USE_QT true)
set (USE_NETWORK true)
include ("${PROJECT_SOURCE_DIR}/../common/CMakeLists.txt")

include_directories(${PROJECT_BINARY_DIR})

qt4_wrap_cpp ( MOC_SRCS
	JogDial.hpp
	LogControl.hpp
	FieldView.hpp
	TreeModel.hpp
)

set (SRCS
	LogFile.cpp
	JogDial.cpp
	LogControl.cpp
	FieldView.cpp
	TreeModel.cpp
	TreeView.cpp
		
	drawing/Elements.cpp
)

add_library (log ${SRCS} ${MOC_SRCS})

add_executable (controlTest EXCLUDE_FROM_ALL test/controlTest.cpp)
target_link_libraries(controlTest log ${QT_LIBRARIES})

add_executable (test EXCLUDE_FROM_ALL test/test.cpp)
target_link_libraries(test log ${QT_LIBRARIES})

