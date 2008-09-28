project (soccsim)
cmake_minimum_required(VERSION 2.6)

# default project build flags
add_definitions (-Werror -Wall)

# use common stuff
include ("../common/CMakeLists.txt")

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
set (QT_USE_QTNETWORK true)
set (QT_USE_QTOPENGL true)
find_package ( Qt4 REQUIRED )
include_directories(${PROJECT_SOURCE_DIR})
include_directories(${PROJECT_BINARY_DIR})
include ( ${QT_USE_FILE} )

qt4_wrap_cpp ( MOC_SRCS
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
endif ()

add_executable (${PROJECT_NAME}
	main.cpp
	Viewer.cpp
	
	## environment
	Physics/Env.cpp
	Physics/Entity.cpp
	Physics/Ball.cpp
	Physics/Field.cpp
	Physics/Robot.cpp
	Physics/MemoryStream.cpp
	
	${SRCS} ${MOC_SRCS} ${UI_HDRS} ${NxuStream2SRC}
)

# if physx had flags to set, set them
if (DEFINED PHYSX_LINK_FLAGS)
	set_target_properties (${PROJECT_NAME} PROPERTIES LINK_FLAGS ${PHYSX_LINK_FLAGS})
endif ()

target_link_libraries (${PROJECT_NAME} NxCooking PhysXLoader pthread dl QtOpenGL QtCore)
