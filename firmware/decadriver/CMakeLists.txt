# set the arm toolchain and include it so we can use it
set(CMAKE_TOOLCHAIN_FILE ${ARM_TOOLCHAIN_FILE})
include(${ARM_TOOLCHAIN_FILE})

# Set compiler and linker flags
set(CMAKE_CXX_FLAGS         ${MBED_CMAKE_CXX_FLAGS}         )
set(CMAKE_C_FLAGS           ${MBED_CMAKE_C_FLAGS}           )
set(CMAKE_EXE_LINKER_FLAGS  ${MBED_CMAKE_EXE_LINKER_FLAGS}  )

# The library files to compile
list(APPEND decadriver_SRC "${CMAKE_CURRENT_SOURCE_DIR}/deca_device.cpp")
list(APPEND decadriver_SRC "${CMAKE_CURRENT_SOURCE_DIR}/deca_params_init.cpp")
list(APPEND decadriver_SRC "${CMAKE_CURRENT_SOURCE_DIR}/deca_range_tables.cpp")
list(APPEND decadriver_SRC "${CMAKE_CURRENT_SOURCE_DIR}/deca_spi.cpp")
list(APPEND decadriver_SRC "${CMAKE_CURRENT_SOURCE_DIR}/deca_sleep.cpp")
list(APPEND decadriver_SRC "${CMAKE_CURRENT_SOURCE_DIR}/deca_mutex.cpp")

# generate a list of directories that we need to include from the values set above
set(decadriver_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR})

# build the 'decadriver' library that contains decawave's api
add_library(decadriver ${decadriver_SRC})
target_include_directories(decadriver PUBLIC ${CMAKE_CURRENT_BINARY_DIR})

# depends on the mbed libraries and all of the accessory libraries
# add_dependencies(decadriver mbed_libraries ${MBED_ASSEC_LIBS_DEPENDS})
add_dependencies(decadriver common2015)

target_link_libraries(decadriver ${MBED_LIBRARY})
target_link_libraries(decadriver common2015)

target_include_directories(decadriver PUBLIC ${decadriver_INCLUDE_DIRS})

# only build firmware library if required
set_target_properties(decadriver PROPERTIES EXCLUDE_FROM_ALL TRUE)
