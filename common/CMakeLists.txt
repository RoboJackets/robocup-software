find_package ( Qt4 REQUIRED )
set (QT_USE_QTNETWORK true)
include ( ${QT_USE_FILE} )

#get the directory of this file
get_filename_component(common_include_dir ${CMAKE_CURRENT_LIST_FILE} PATH)

#set the include dir to this file's directory
#message( STATUS ${common_include_dir})
include_directories(${common_include_dir})

# this will run MOC on the specified header files
qt4_wrap_cpp (common_MOC_SRCS
        )

macro (prepend_list aList aString)
    set (temp_list)
    
    foreach (list_item ${${aList}})
        list(APPEND temp_list ${aString}/${list_item})
    endforeach(list_item)
    
    set (${aList} ${temp_list})
endmacro(prepend_list)

if (USE_PACKET)
    SET(libpacket_SRCS
        Packet/UdpSocket.cpp
        )
        
    prepend_list(libpacket_SRCS ${common_include_dir})
#   message( STATUS ${libpacket_SRCS}) 
    message( STATUS "Using the Packet library") 
    
    add_library (packet ${libpacket_SRCS} ${common_MOC_SRCS}) 
    
    set (PACKET packet)
endif(USE_PACKET)
 
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
    prepend_list(libgeometry_SRCS ${common_include_dir})
# message( STATUS ${libgeometry_SRCS}) 
    message( STATUS "Using the Geometry library") 
    
    add_library (geometry ${libgeometry_SRCS} ${common_MOC_SRCS})
    
    set (GEOMETRY geometry)
endif(USE_GEOMETRY)
