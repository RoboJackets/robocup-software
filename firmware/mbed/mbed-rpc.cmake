# The cmake build file that sets up everything for building the mbed RPC
# classes for use along with the 'mbed_libraries' library.

# select the source files for the RPC classes
file(GLOB RPC_SRC "${MBED_RPC_PATH}/*.cpp")

# add a library for building the RPC classes
add_library(rpc_library ${RPC_SRC})

# we add dependencies for the source files since those
# are downloaded with the mbed_libraries external project
add_dependencies(rpc_library mbed_libraries ${RPC_SRC})

# pass the mbed_libraries target to the linker
target_link_libraries(rpc_library mbed_libraries)

# include the directory for anything that depends on this target library
target_include_directories(rpc_library PUBLIC ${MBED_RPC_PATH})

# only build if required
set_target_properties(rpc_library PROPERTIES EXCLUDE_FROM_ALL TRUE)

# add the external project's path/src info into the accessory library lists
set(MBED_ASSEC_LIBS_INCLUDES    ${MBED_ASSEC_LIBS_INCLUDES} ${SOURCE_DIR}       )
set(MBED_ASSEC_LIBS_SRCS        ${MBED_ASSEC_LIBS_SRCS}     ${MBED_RPC_PATH}    )
set(MBED_ASSEC_LIBS_DEPENDS     ${MBED_ASSEC_LIBS_DEPENDS}  rpc_library         )
