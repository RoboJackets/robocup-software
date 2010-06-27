#get the directory of this file
get_filename_component(common_include_dir ${CMAKE_CURRENT_LIST_FILE} PATH)

#set the include dir to this file's directory
#message( STATUS ${common_include_dir})
include_directories(${common_include_dir})

if (NOT TARGET GeneratePackets)
	macro (prepend_list aList aString)
		set (temp_list)
		
		foreach (list_item ${${aList}})
			list(APPEND temp_list ${aString}/${list_item})
		endforeach(list_item)
		
		set (${aList} ${temp_list})
	endmacro(prepend_list)
	
	set (PacketDir ${common_include_dir}/packets)
	set (Packets
		Vision
		RadioTx
		RadioRx
		Point
		Segment
		Polygon
		Referee
		GameState
		LogFrame
		SimCommand
		MotionCmd
	)
	
	foreach (packet ${Packets})
		set (xmlFile ${PacketDir}/${packet}.xml)
		set (header ${packet}.hpp)
		
		add_custom_command(OUTPUT ${header}
				COMMAND ${common_include_dir}/SerDes/gen_cpp ${xmlFile} > ${header}
				DEPENDS ${xmlFile} ${common_include_dir}/SerDes/gen_cpp
				
				VERBATIM
		)
		
		list (APPEND PacketHeaders ${PROJECT_BINARY_DIR}/${header})
	endforeach()
	
	add_custom_target(GeneratePackets
		DEPENDS ${PacketHeaders}
		VERBATIM
	)
endif()

if (USE_QT)
	add_definitions (-DQT_NO_KEYWORDS)
	set (QT_USE_QTNETWORK true)
	set (QT_USE_QTOPENGL true)
	set (QT_USE_QTXML true)
	find_package ( Qt4 REQUIRED )
	include_directories(${PROJECT_SOURCE_DIR})
	include_directories(${PROJECT_BINARY_DIR})
	include ( ${QT_USE_FILE} )
endif(USE_QT)

if (USE_GEOMETRY AND NOT TARGET geometry)
	SET(libgeometry_SRCS 
		Point.cpp 
		Circle.cpp 
		Line.cpp 
		Rect.cpp 
		Segment.cpp 
		TransformMatrix.cpp
		Polygon.cpp
	)

	prepend_list(libgeometry_SRCS "Geometry2d/")
	prepend_list(libgeometry_SRCS ${common_include_dir})
	message( STATUS "Using the Geometry library") 
	
	add_library (geometry ${libgeometry_SRCS})
	
	set (GEOMETRY_LIBRARY geometry)
endif()

if (USE_NETWORK AND NOT TARGET network)
	include_directories(${common_include_dir}/SerDes)
	
	set (network_srcs
		Sender.cpp
		Receiver.cpp
		Network.cpp
	)

	prepend_list(network_srcs "${common_include_dir}/Network/")
	message( STATUS "Using the Network library") 
	
	add_library (network ${network_srcs})
	add_dependencies (network GeneratePackets)
	set (NETWORK_LIBRARY network)
endif()

