project (LogModule)
cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

add_definitions(-O2 -Wall -g3)

set (USE_QT true)
set (USE_NETWORK true)
set (USE_GEOMETRY true)
set (USE_FRAMEWORK true)
set (USE_CONFIG true)
include ("${PROJECT_SOURCE_DIR}/../common/CMakeLists.txt")

include_directories(${PROJECT_BINARY_DIR})

qt4_wrap_cpp ( MOC_SRCS
	LogControl.hpp
	FieldView.hpp
	TreeModel.hpp
)

set (SRCS
	LogFile.cpp
	LogControl.cpp
	FieldView.cpp
	TreeModel.cpp
	TreeView.cpp
	LogModule.cpp	
	
	drawing/Elements.cpp
)

add_library (log ${SRCS} ${MOC_SRCS})

add_dependencies (log gen_tree)

if (NOT TARGET gen_tree)	
    #common CMakeLists.txt file contains the list of packets
    
	foreach (packet ${Packets})
	    set (xmlFile ${PacketDir}/${packet}.xml)
	    set (header tree_${packet}.hpp)
	    
	    add_custom_command(OUTPUT ${header}
	            COMMAND ${PROJECT_SOURCE_DIR}/gen_tree ${xmlFile} > ${header}
	            DEPENDS ${xmlFile} ${PROJECT_SOURCE_DIR}/gen_tree
				
	            VERBATIM
	    )
	    
	    list (APPEND GenHeaders ${PROJECT_BINARY_DIR}/${header})
	endforeach()
	
	add_custom_target(gen_tree
	    DEPENDS ${GenHeaders}
	    VERBATIM
	)
endif()
