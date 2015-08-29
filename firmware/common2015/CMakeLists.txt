# set the arm toolchain and include it so we can use it
set(CMAKE_TOOLCHAIN_FILE ${ARM_TOOLCHAIN_FILE})
include(${ARM_TOOLCHAIN_FILE})
include(${MBED_ASSEC_LIBS})

# Set compiler and linker flags
set(CMAKE_CXX_FLAGS ${MBED_CMAKE_CXX_FLAGS})
set(CMAKE_C_FLAGS ${MBED_CMAKE_C_FLAGS})
set(CMAKE_EXE_LINKER_FLAGS ${MBED_CMAKE_EXE_LINKER_FLAGS})

file(GLOB_RECURSE common2015_SRC "*.cpp")
list(REMOVE_ITEM common2015_SRC ${CMAKE_CURRENT_LIST_DIR}/ExampleTest.cpp)
list(APPEND common2015_SRC ${SWSPI_SRC})

# set the names of each directory and then set the names of the subdirectories within each one
set(DRIVERS_ROOT_NAME   drivers)
set(MODULES_ROOT_NAME   modules)
set(UTILS_ROOT_NAME     utils)
set(DRIVERS             cc1101 cc1201 cc1201/cfg)
set(MODULES             CommModule CommLink Console)
set(UTILS               assert fp16 logger numparser rtos-mgmt)

# generate a list of directories that we need to include from the values set above
set(common2015_INCLUDE_DIRS 
    ${CMAKE_CURRENT_SOURCE_DIR}/${DRIVERS_ROOT_NAME} 
    ${CMAKE_CURRENT_SOURCE_DIR}/${MODULES_ROOT_NAME} 
    ${CMAKE_CURRENT_SOURCE_DIR}/${UTILS_ROOT_NAME}
)
foreach(driver_subdir ${DRIVERS})
    list(APPEND common2015_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/${DRIVERS_ROOT_NAME}/${driver_subdir})
endforeach()
foreach(module_subdir ${MODULES})
    list(APPEND common2015_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/${MODULES_ROOT_NAME}/${module_subdir})
endforeach()
foreach(util_subdir ${UTILS})
    list(APPEND common2015_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/${UTILS_ROOT_NAME}/${util_subdir})
endforeach()

# append the asscessory library include directories
set(common2015_INCLUDE_DIRS ${common2015_INCLUDE_DIRS} ${MBED_ASSEC_LIBS_PATH})

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
add_dependencies(common2015 mbed_libraries ${MBED_ASSEC_LIBS_DEPENDS})

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

set(MBED_TEST_UNIT 1)
# custom target for creating a .bin file from an elf binary
add_custom_target(mbed-test
    COMMAND sudo python2 ${PY_TOOLS_DIR}/workspace_tools/make.py --mcu=${MBED_TARGET} --tool=GCC_ARM
        --disk=/media/jonathan/MBED --serial=/dev/mbed --verbose -p ${MBED_TEST_UNIT} --jobs=0 --baud=9600
    DEPENDS mbed_libraries
    COMMENT "building mbed test binary"
)
set_target_properties(mbed-test PROPERTIES EXCLUDE_FROM_ALL TRUE)
