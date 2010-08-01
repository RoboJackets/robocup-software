#get the directory of this file
get_filename_component(common_include_dir ${CMAKE_CURRENT_LIST_FILE} PATH)

#set the include dir to this file's directory
include_directories(${common_include_dir})

# Build protobuf files
add_subdirectory (${common_include_dir}/protobuf ${PROJECT_BINARY_DIR}/protobuf)

macro (prepend_list aList aString)
	set (temp_list)
	
	foreach (list_item ${${aList}})
		list(APPEND temp_list ${aString}/${list_item})
	endforeach(list_item)
	
	set (${aList} ${temp_list})
endmacro(prepend_list)
	
# Qt
add_definitions (-DQT_NO_KEYWORDS)
set (QT_USE_QTNETWORK true)
set (QT_USE_QTOPENGL true)
set (QT_USE_QTXML true)
find_package ( Qt4 REQUIRED )
include_directories(${PROJECT_SOURCE_DIR})
include_directories(${PROJECT_BINARY_DIR})
include ( ${QT_USE_FILE} )

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

add_library (geometry ${libgeometry_SRCS})
target_link_libraries (geometry packets)

set (GEOMETRY_LIBRARY geometry)

