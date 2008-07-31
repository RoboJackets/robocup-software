
# some of the syntax requires the 2.6 version of cmake
cmake_minimum_required(VERSION 2.6)

find_package ( Qt4 REQUIRED )
set (QT_USE_QTNETWORK true)
include ( ${QT_USE_FILE} )

#get the directory of this file
get_filename_component(common_dir ${CMAKE_CURRENT_LIST_FILE} PATH)

#set the include dir to this file's directory
#message( STATUS ${common_include_dir})
include_directories(${common_dir})

# this will run MOC on the specified header files
qt4_wrap_cpp (common_MOC_SRCS
        )

macro (prepend_list aList aString)
    set (temp_list)
    
    foreach (list_item ${${aList}})
        list(APPEND temp_list ${aString}/${list_item})
    endforeach()
    
    set (${aList} ${temp_list})
    
endmacro ()

if (USE_PACKET)
    SET(libpacket_SRCS
        Packet/UdpSocket.cpp
        )
        
    prepend_list(libpacket_SRCS ${common_dir})
	#message( STATUS ${libpacket_SRCS}) 
	#message( STATUS "Using the Packet library") 
    
    add_library (packet ${libpacket_SRCS} ${common_MOC_SRCS}) 
    
    set (PACKET packet)
    
	# input xml packets
	set (Packets
		CommData
		LogMotion
		MotionCmd
		Ref
		RobotStatus
		SkillCmd
		SkillStatus
		VisionData
		SimCmd
	)
	
	set (utilDir ${common_dir}/../util)
	set (PacketGenDir ${utilDir}/PacketGenerator/bin/packetGenerator)
	set (PacketDir ${common_dir}/Packet/packets)
	
	add_subdirectory(${utilDir}/PacketGenerator ${PROJECT_BINARY_DIR})
	
	foreach (packet ${Packets})
		set (xmlFile ${PacketDir}/${packet}.xml)
		set (header ${packet})
		
		add_custom_command(OUTPUT 
			${header}
			COMMAND ${PROJECT_BINARY_DIR}/packetGenerator ${xmlFile}
			DEPENDS ${xmlFile}
			VERBATIM
		)
		
		list (APPEND PacketHeaders ${PROJECT_BINARY_DIR}/${header})
	endforeach()
	
	add_custom_target(GeneratePackets
		DEPENDS
		${PacketHeaders}
		VERBATIM
	)
	
	add_dependencies (GeneratePackets packetGenerator)
    
endif ()
 
if (USE_GEOMETRY)
    SET(libgeometry_SRCS 
        Point2d.cpp 
        Circle2d.cpp 
        Line2d.cpp 
        Rect.cpp 
        Segment.cpp 
        TransformMatrix.cpp
        )

    prepend_list(libgeometry_SRCS "Geometry/")
    prepend_list(libgeometry_SRCS ${common_dir})
# message( STATUS ${libgeometry_SRCS}) 
    message( STATUS "Using the Geometry library") 
    
    add_library (geometry ${libgeometry_SRCS} ${common_MOC_SRCS})
    
    set (GEOMETRY geometry)
endif ()
