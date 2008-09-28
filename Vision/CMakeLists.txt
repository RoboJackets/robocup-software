set(PROJECT vision)
cmake_minimum_required(VERSION 2.6)

if (${UNIX})
	add_definitions(-D_LINUX)
endif ()

set(SRCS
    main.cpp
    Config_File.cpp
    Camera_Thread.cpp
    Camera_Window.cpp
    GL_Camera_View.cpp
    Image.cpp
    Image_Texture.cpp
    Setup_Mode.cpp
    Distortion_Setup.cpp
    Transform_Setup.cpp
    Colorspace_Window.cpp
    Colorspace_View.cpp
    
    camera/test/Dummy.cpp 
    
    camera/file/File.cpp
    camera/file/File_Control.cpp
    
    camera/dcam/DCam.cpp
    camera/dcam/DCam_Config.cpp
    
    camera/prosilica/Prosilica.cpp 
    camera/prosilica/AttrBase.cpp
    camera/prosilica/AttrCommand.cpp 
    camera/prosilica/AttrEnum.cpp 
    camera/prosilica/AttrFloat.cpp 
    camera/prosilica/AttrString.cpp 
    camera/prosilica/AttrUint32.cpp
    
    vision/Colors.cpp
    vision/Colorseg.cpp
    vision/Spanner.cpp
    vision/Process.cpp
    vision/Transform.cpp
    vision/Distortion.cpp
    
    vision/identification/Dot_ID.cpp
    vision/identification/Vector_ID.cpp
    vision/identification/Offset3_ID.cpp
)

set(UIS
    ui/camera_window.ui
    ui/colorspace_window.ui
    ui/pattern_window.ui
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
qt4_add_resources(RC_SRCS ui/vision.qrc)

if (${CMAKE_SIZEOF_VOID_P} EQUAL 8)
	add_definitions(-D_x64)
	set (prosilica_lib_dir x64)
else ()
	add_definitions(-D_x86)
	set (prosilica_lib_dir x86)
endif ()

set (prosilica_dir ${PROJECT_SOURCE_DIR}/lib/prosilica)
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
