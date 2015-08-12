# set the arm toolchain and include it so we can use it
set(CMAKE_TOOLCHAIN_FILE ${ARM_TOOLCHAIN_FILE})
include(${ARM_TOOLCHAIN_FILE})

# Set compiler and linker flags
set(CMAKE_CXX_FLAGS ${MBED_CMAKE_CXX_FLAGS})
set(CMAKE_C_FLAGS ${MBED_CMAKE_C_FLAGS})
set(CMAKE_EXE_LINKER_FLAGS ${MBED_CMAKE_EXE_LINKER_FLAGS})

set(common2015_SRC
    # "drivers/radio/radio.cpp"
    "drivers/cc1101/CC1101.cpp"
    "drivers/cc1201/CC1201.cpp"

    "modules/Console/Console.cpp"
    "modules/CommLink/CommLink.cpp"
    "modules/CommModule/CommModule.cpp"
    
    "utils/fp16/fp16.cpp"
    "utils/assert/assert.cpp"
    "utils/logger/logger.cpp"
    "utils/numparser/numparser.cpp"
    "utils/rtos-mgmt/helper-funcs.cpp"
    "utils/rtos-mgmt/thread-helper.cpp"
)

# set the names of each directory and then set the names of the subdirectories within each one
set(DRIVERS_ROOT_NAME   drivers)
set(MODULES_ROOT_NAME   modules)
set(UTILS_ROOT_NAME     utils)
set(DRIVERS             cc1101 cc1201 cc1201/cfg radio ring-buffer)
set(MODULES             CommModule CommLink Console)
set(UTILS               assert fp16 logger numparser rtos-mgmt watchdog)

# generate a list of directories that we need to include from the values set above
set(common2015_INCLUDE_DIRS ${DRIVERS_ROOT_NAME} ${MODULES_ROOT_NAME} ${UTILS_ROOT_NAME})

foreach(driver_subdir ${DRIVERS})
    list(APPEND common2015_INCLUDE_DIRS ${DRIVERS_ROOT_NAME}/${driver_subdir})
endforeach()
foreach(module_subdir ${MODULES})
    list(APPEND common2015_INCLUDE_DIRS ${MODULES_ROOT_NAME}/${module_subdir})
endforeach()
foreach(util_subdir ${UTILS})
    list(APPEND common2015_INCLUDE_DIRS ${UTILS_ROOT_NAME}/${util_subdir})
endforeach()

# find all of the header files
file(GLOB_RECURSE common2015_HEADERS "*.hpp")

# tell CMake that the obj files all come from the ExternalProject
# otherwise it'll complain that the files can't be found
foreach(mbed_obj ${MBED_OBJECTS})
    add_custom_command(
        OUTPUT      ${mbed_obj}
        DEPENDS     mbed_libraries
        COMMAND     ""
    )
endforeach()

# build the 'common2015' library that contains all the MBED stuff needed for
# both the base station and robot firmware
add_library(common2015 ${common2015_SRC} ${CMAKE_BINARY_DIR}/common/git_version.cpp)
add_dependencies(common2015 mbed_libraries)

target_link_libraries(common2015 ${MBED_OBJECTS})
target_link_libraries(common2015 ${MBED_LIBS})

target_include_directories(common2015 PUBLIC ${common2015_INCLUDE_DIRS})

# only build firmware library if required
set_target_properties(common2015 PROPERTIES EXCLUDE_FROM_ALL TRUE)

# See common/CMakeLists for more info on how this works
add_custom_command(
    OUTPUT ${CMAKE_BINARY_DIR}/common/git_version.cpp
    DEPENDS git_version
)
