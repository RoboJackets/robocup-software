# set the arm toolchain and include it so we can use it
set(CMAKE_TOOLCHAIN_FILE ${ARM_TOOLCHAIN_FILE})
include(${ARM_TOOLCHAIN_FILE})

# Set compiler and linker flags
set(CMAKE_CXX_FLAGS         ${MBED_CMAKE_CXX_FLAGS}         )
set(CMAKE_C_FLAGS           ${MBED_CMAKE_C_FLAGS}           )
set(CMAKE_EXE_LINKER_FLAGS  ${MBED_CMAKE_EXE_LINKER_FLAGS}  )

file(GLOB_RECURSE   common2015_SRC  "*.cpp")
list(REMOVE_ITEM    common2015_SRC  "${CMAKE_CURRENT_LIST_DIR}/ExampleTest.cpp")

# append all of the accessory library sources
list(APPEND common2015_SRC ${MBED_ASSEC_LIBS_SRCS})

# set the names of each directory and then set the names of the subdirectories within each one
set(DRIVERS_ROOT_NAME   drivers)
set(MODULES_ROOT_NAME   modules)
set(UTILS_ROOT_NAME     utils)
# subdirectories
set(DRIVERS             cc1101 cc1201 cc1201/cfg rtos-i2c software-spi)
set(MODULES             CommModule CommLink Console)
set(UTILS               assert logger numparser rtos-mgmt)

# generate a list of directories that we need to include from the values set above
set(common2015_INCLUDE_DIRS 
    ${CMAKE_CURRENT_SOURCE_DIR}/${DRIVERS_ROOT_NAME} 
    ${CMAKE_CURRENT_SOURCE_DIR}/${MODULES_ROOT_NAME} 
    ${CMAKE_CURRENT_SOURCE_DIR}/${UTILS_ROOT_NAME}
)

include(ShowVars)
show_vars()

# append all of the common2015 subdirectories listed in the variables above
foreach(driver_subdir ${DRIVERS})
    list(APPEND common2015_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/${DRIVERS_ROOT_NAME}/${driver_subdir})
endforeach()
foreach(module_subdir ${MODULES})
    list(APPEND common2015_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/${MODULES_ROOT_NAME}/${module_subdir})
endforeach()
foreach(util_subdir ${UTILS})
    list(APPEND common2015_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/${UTILS_ROOT_NAME}/${util_subdir})
endforeach()

# append all of the accessory library include directories
list(APPEND common2015_INCLUDE_DIRS ${MBED_ASSEC_LIBS_INCLUDES})

# tell cmake that all of the source files aren't here yet
set_source_files_properties(
    ${MBED_ASSEC_LIBS_SRCS} PROPERTIES
    GENERATED               TRUE
    DEPENDS                 ${MBED_ASSEC_LIBS_DEPENDS}
)

# build the 'common2015' library that contains all the MBED stuff needed for
# both the base station and robot firmware
add_library(common2015 ${common2015_SRC} ${CMAKE_BINARY_DIR}/common/git_version.cpp)

# depends on the mbed libraries and all of the accessory libraries
add_dependencies(common2015 mbed_libraries ${MBED_ASSEC_LIBS_DEPENDS})

target_link_libraries(common2015 ${MBED_LIBRARY})

target_include_directories(common2015 PUBLIC ${common2015_INCLUDE_DIRS})

# only build firmware library if required
set_target_properties(common2015 PROPERTIES EXCLUDE_FROM_ALL TRUE)

# See common/CMakeLists for more info on how this works
add_custom_command(
    OUTPUT ${CMAKE_BINARY_DIR}/common/git_version.cpp
    DEPENDS git_version
)
