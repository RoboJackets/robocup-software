
include(../mbed/mbed_executable.cmake)


file(GLOB_RECURSE   common2015_SRC  "*.cpp")
list(REMOVE_ITEM    common2015_SRC  "${CMAKE_CURRENT_LIST_DIR}/ExampleTest.cpp")

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

# Include the arm toolchain for gcc
include(${ARM_TOOLCHAIN_FILE})
rj_mbed_env()

# build the 'common2015' library that contains all the MBED stuff needed for
# both the base station and robot firmware
rj_add_mbed_library(common2015 ${common2015_SRC} ${CMAKE_BINARY_DIR}/common/git_version.cpp)
target_link_libraries(common2015 modserial)
target_link_libraries(common2015 burstspi)
# target_link_libraries(common2015 moddma)

target_link_libraries(common2015 ${MBED_LIBRARY})

target_include_directories(common2015 PUBLIC ${common2015_INCLUDE_DIRS})

# See common/CMakeLists for more info on how this works
add_custom_command(
    OUTPUT ${CMAKE_BINARY_DIR}/common/git_version.cpp
    DEPENDS git_version
)
