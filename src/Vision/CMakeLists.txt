set(PROJECT vision)

if (${UNIX})
	add_definitions(-D_LINUX)
else (${UNIX})
#not _LINUX...who cares
#add_definitions(-D_LINUX)
endif (${UNIX})

if (CMAKE_SIZEOF_VOID_P EQUAL 4)
	add_definitions(-D_x86)
	set (prosilica_lib_dir x86)
else (CMAKE_SIZEOF_VOID_P EQUAL 8)
	add_definitions(-D_x64)
	set (prosilica_lib_dir x64)
endif (CMAKE_SIZEOF_VOID_P EQUAL 4)

set(SRCS
    main.cpp
    Config_File.cpp Camera_Thread.cpp Camera_Window.cpp
    GL_Camera_View.cpp Image.cpp Image_Texture.cpp
    Setup_Mode.cpp Distortion_Setup.cpp Transform_Setup.cpp
    Pattern_Window.cpp Pattern_View.cpp
    Colorspace_Window.cpp Colorspace_View.cpp
    
    camera/Dummy.cpp camera/File.cpp camera/File_Control.cpp camera/DCam.cpp
    camera/DCam_Config.cpp
    camera/Prosilica.cpp 
    camera/AttrBase.cpp
    camera/AttrCommand.cpp 
    camera/AttrEnum.cpp 
    camera/AttrFloat.cpp 
    camera/AttrString.cpp 
    camera/AttrUint32.cpp
    
    vision/Colors.cpp
    vision/Colorseg.cpp vision/Spanner.cpp vision/Tracker.cpp vision/Process.cpp
    vision/Dot_ID.cpp vision/Vector_ID.cpp vision/Offset3_ID.cpp vision/Sender.cpp
    vision/Transform.cpp vision/Distortion.cpp
)

set(UIS
    camera_window.ui
    colorspace_window.ui
    pattern_window.ui
    camera/file_control.ui
)

project(${PROJECT})

set (USE_PACKET true)
set (USE_GEOMETRY true)
include ("../common/CMakeLists.txt")

add_definitions(-O2 -Wall -g3 -funroll-loops -Wno-deprecated-declarations)

find_package(Qt4 REQUIRED)
set(QT_USE_QTOPENGL TRUE)
set(QT_USE_QTXML TRUE)
set(QT_USE_QTNETWORK TRUE)
include_directories(${CMAKE_CURRENT_BINARY_DIR})
include(${QT_USE_FILE})
qt4_automoc(${SRCS})
qt4_wrap_ui(UIHS ${UIS})
qt4_add_resources(RC_SRCS vision.qrc)

set (prosilica_dir ${PROJECT_SOURCE_DIR}/camera/prosilica_lib)
include_directories(${prosilica_dir}/include)
link_directories(${prosilica_dir}/lib/${prosilica_lib_dir}/4.1/)

add_executable(${PROJECT} ${SRCS} ${UIHS} ${RC_SRCS})
target_link_libraries(${PROJECT} ${QT_LIBRARIES})

find_package(PkgConfig REQUIRED)
pkg_search_module(LIBAVFORMAT REQUIRED libavformat)
include_directories(${LIBAVFORMAT_INCLUDE_DIRS})
link_directories(${LIBAVFORMAT_LIBRARY_DIRS})
target_link_libraries(${PROJECT} ${LIBAVFORMAT_LIBRARIES})

find_package(GLUT REQUIRED)
include_directories(${GLUT_INCLUDE_DIR})
target_link_libraries(${PROJECT} ${GLUT_LIBRARIES})

target_link_libraries(${PROJECT} dc1394 PvAPI)

add_custom_target(run ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT}
	WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
	VERBATIM)
add_dependencies(run ${PROJECT})

add_custom_target(run_video ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT} config/video.xml
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    VERBATIM)
add_dependencies(run_video ${PROJECT})

target_link_libraries(${PROJECT} ${PACKET} ${GEOMETRY})
