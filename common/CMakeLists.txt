#get the directory of this file
get_filename_component(common_include_dir ${CMAKE_CURRENT_LIST_FILE} PATH)

#set the include dir to this file's directory
include_directories(${common_include_dir})

macro (prepend_list aList aString)
	set (temp_list)
	
	foreach (list_item ${${aList}})
		list(APPEND temp_list ${aString}/${list_item})
	endforeach(list_item)
	
	set (${aList} ${temp_list})
endmacro(prepend_list)
	
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

