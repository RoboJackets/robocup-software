project (soccsim)
cmake_minimum_required(VERSION 2.6)

# default project build flags
add_definitions (-Werror -Wall)

#set (CMAKE_VERBOSE_MAKEFILE on)
# use common stuff
set (USE_NETWORK true)
set (USE_GEOMETRY true)
include ("${PROJECT_SOURCE_DIR}/../common/CMakeLists.txt")

# PhysX Setup
set (PHYSX_VERSION 2.8.1)
set (PHYSX_SDK_PATH /usr/include/PhysX/v${PHYSX_VERSION}/SDKs)
set (PHYSX_LIB_PATH /usr/lib/PhysX/v${PHYSX_VERSION})

# Required includes for PhysX
include_directories (${PHYSX_SDK_PATH}/Physics/include/)
include_directories (${PHYSX_SDK_PATH}/Foundation/include/)
include_directories (${PHYSX_SDK_PATH}/PhysXLoader/include/)
include_directories (${PHYSX_SDK_PATH}/Cooking/include/)

# PhysX library path
link_directories(${PHYSX_LIB_PATH})

# PhysX defines
add_definitions (-DNX32)
add_definitions (-DLINUX)

# disable fluids, otherwise PhysX doesn't work in linux
add_definitions (-DNX_DISABLE_FLUIDS)

# setup QT
add_definitions (-DQT_NO_KEYWORDS)
set (USE_QT true)
set (QT_USE_QTNETWORK true)
set (QT_USE_QTOPENGL true)
set (QT_USE_QTXML true)
find_package ( Qt4 REQUIRED )
include_directories(${PROJECT_SOURCE_DIR})
include_directories(${PROJECT_BINARY_DIR})
include ( ${QT_USE_FILE} )

qt4_wrap_cpp ( MOC_SRCS
	Physics/Env.hpp
	InputHandler.hpp
	)

# this will run UIC on the .ui files
qt4_wrap_ui ( UI_HDRS
	)

# list any resource files
qt4_add_resources ( RC_SRCS
	)

# 64bit machines need to cross compile in 32bit mode
# because the PhysX library only has 32bit linux binaries
if (${CMAKE_SIZEOF_VOID_P} EQUAL 8)
	add_definitions(-m32)
	set (PHYSX_LINK_FLAGS -m32)
	message (STATUS "Cross Compiling to 32 bit mode")
	message (STATUS "run: 'make linkfix' as root to make required symlinks if needed")

	add_custom_command(OUTPUT "/usr/lib32/libQtOpenGL.so"
		COMMAND ln ARGS -s libQtOpenGL.so.4 /usr/lib32/libQtOpenGL.so )

	add_custom_command(OUTPUT "/usr/lib32/libQtGui.so"
		COMMAND ln ARGS -s libQtGui.so.4 /usr/lib32/libQtGui.so )

	add_custom_command(OUTPUT "/usr/lib32/libQtXml.so"
		COMMAND ln ARGS -s libQtXml.so.4 /usr/lib32/libQtXml.so )

	add_custom_command(OUTPUT "/usr/lib32/libQtNetwork.so"
		COMMAND ln ARGS -s libQtNetwork.so.4 /usr/lib32/libQtNetwork.so )

	add_custom_command(OUTPUT "/usr/lib32/libGLU.so"
		COMMAND ln ARGS -s libGLU.so.4 /usr/lib32/libGLU.so )

	add_custom_command(OUTPUT "/usr/lib32/libGL.so"
		COMMAND ln ARGS -s libGL.so.4 /usr/lib32/libGL.so )

	add_custom_command(OUTPUT "/usr/lib32/libX11.so"
		COMMAND ln ARGS -s libX11.so.6 /usr/lib32/libX11.so )

	add_custom_command(OUTPUT "/usr/lib32/libXext.so"
		COMMAND ln ARGS -s libXext.so.6 /usr/lib32/libXext.so )

	add_custom_target(linkfix
		DEPENDS /usr/lib32/libQtOpenGL.so
				/usr/lib32/libQtGui.so
				/usr/lib32/libQtNetwork.so
				/usr/lib32/libQtXml.so
				/usr/lib32/libGLU.so
				/usr/lib32/libGL.so
				/usr/lib32/libX11.so
				/usr/lib32/libXext.so
	)
endif ()

add_executable (${PROJECT_NAME}
	main.cpp
	Viewer.cpp
	JoystickInput.cpp
	InputHandler.cpp
	Config.cpp
	Vision.cpp

	## environment
	Physics/Env.cpp
	Physics/Entity.cpp
	Physics/Ball.cpp
	Physics/Field.cpp
	Physics/Robot.cpp
	Physics/MemoryStream.cpp
	${MOC_SRCS} ${UI_HDRS} ${NxuStream2SRC}
)

# if physx had flags to set, set them
if (DEFINED PHYSX_LINK_FLAGS)
	set_target_properties (${PROJECT_NAME} PROPERTIES LINK_FLAGS ${PHYSX_LINK_FLAGS})
endif ()

target_link_libraries (${PROJECT_NAME} NxCooking PhysXLoader pthread dl ${QT_LIBRARIES}  )
target_link_libraries(${PROJECT_NAME} ${NETWORK_LIBRARY})
target_link_libraries(${PROJECT_NAME} ${PACKET_LIBRARY})
#target_link_libraries(${PROJECT_NAME} ${GEOMETRY_LIBRARY})

