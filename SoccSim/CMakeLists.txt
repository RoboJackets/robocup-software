project (soccsim)
cmake_minimum_required (VERSION 2.6 FATAL_ERROR)

# Are we building on a 64-bit system?
if (${CMAKE_SIZEOF_VOID_P} EQUAL 8)
    # Cross compile to 32-bit
    set (CROSS32 1)
    
    # This must be done before add_subdirectory for some libraries
    add_definitions(-m32)
endif ()

# default project build flags
add_definitions (-Werror -Wall -g3)

# use common stuff
include (${PROJECT_SOURCE_DIR}/../common/CMakeLists.txt)

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
link_directories (${PHYSX_LIB_PATH})

# PhysX defines
add_definitions (-DNX32)
add_definitions (-DLINUX)

# disable fluids, otherwise PhysX doesn't work in linux
add_definitions (-DNX_DISABLE_FLUIDS)

set (SRCS
    main.cpp
    Viewer.cpp
    Config.cpp

    ## environment
    Physics/Env.cpp
    Physics/Entity.cpp
    Physics/Ball.cpp
    Physics/Field.cpp
    Physics/Robot.cpp
    Physics/MemoryStream.cpp
)

# Run moc on headers for Q_OBJECTs
qt4_automoc (${SRCS})

# Fix libraries when cross-compiling
if (CROSS32)
    # This is disgusting.  We need a good open-source physics engine.
    file(READ /etc/issue ISSUE)
    if (${ISSUE} MATCHES "^Ubuntu 10\\.04")
        set(COMPAT_SYSTEM lucid_amd64)
        message(STATUS "Compatibility: Compiling on 64-bit Ubuntu 10.04")
    elseif (${ISSUE} MATCHES "^Ubuntu 9\\.")
        set(COMPAT_SYSTEM jaunty_amd64)
        message(STATUS "Compatibility: Compiling on 64-bit Ubuntu 9.xx")
    
        # The ia32-libs package does not contain libQtOpenGL.so, so we provide it.
        # Remove the old libQtOpenGL from the linker command line so the linker doesn't look for the system (64-bit) version.
        list(REMOVE_ITEM QT_LIBRARIES ${QT_QTOPENGL_LIBRARY})
        list(APPEND QT_LIBRARIES ${PROJECT_SOURCE_DIR}/lib/32-on-64/${COMPAT_SYSTEM}/libQtOpenGL.so)
    endif ()

    if (DEFINED COMPAT_SYSTEM)
        # This must be done before add_executable
        link_directories(${CMAKE_SOURCE_DIR}/lib/32-on-64/${COMPAT_SYSTEM})
    endif ()
endif ()

add_executable (${PROJECT_NAME}
    ${SRCS} ${NxuStream2SRC}
)

if (CROSS32)
    set_target_properties (${PROJECT_NAME} PROPERTIES LINK_FLAGS -m32)
endif ()

target_link_libraries (${PROJECT_NAME} NxCooking PhysXLoader ${QT_LIBRARIES})
target_link_libraries (${PROJECT_NAME} ${GEOMETRY_LIBRARY})
target_link_libraries (${PROJECT_NAME} packets)

# Copy the executable out to project /bin folder
#file(COPY ./bin/${PROJECT_NAME} DESTINATION ../../bin/)
