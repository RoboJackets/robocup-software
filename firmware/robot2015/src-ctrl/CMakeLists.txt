# this file came from https://github.com/uwearzt/mbed-cmake
# and was modified by Justin Buchanan
# 
# The end result of this file is the 'robot2015' target
# building this target ouputs an MBED-runnable file at run/robot2015.bin, which can be copied onto the MBED
set(CMAKE_TOOLCHAIN_FILE ${ARM_TOOLCHAIN_FILE})
include(${ARM_TOOLCHAIN_FILE})

# # Find the assembly source files and make sure they're compiled using the C compiler
file(GLOB_RECURSE asm_SRC "*.S")
set_property(SOURCE ${asm_SRC} PROPERTY LANGUAGE C)

include(../../mbed/mbed_executable.cmake)

# set the names of each directory and then set the names of the subdirectories within each one
set(DRIVERS_ROOT_NAME   ${CMAKE_CURRENT_SOURCE_DIR}/drivers)
set(MODULES_ROOT_NAME   ${CMAKE_CURRENT_SOURCE_DIR}/modules)
set(CONFIG_ROOT_NAME    ${CMAKE_CURRENT_SOURCE_DIR}/config)
set(CONFIG_ROOT_NAME    ${CONFIG_ROOT_NAME} PARENT_SCOPE)
set(DRIVERS             avr-isp buzzer dma ds2411 fpga mcp23017 mpu-6050 kicker-board ws2811)
set(MODULES             commands motors)

# generate a list of directories that we need to include from the values set above
set(ROBOT2015_ELF_INCLUDES ${CONFIG_ROOT_NAME} ${DRIVERS_ROOT_NAME} ${MODULES_ROOT_NAME})
foreach(driver_subdir ${DRIVERS})
    list(APPEND ROBOT2015_ELF_INCLUDES ${DRIVERS_ROOT_NAME}/${driver_subdir})
endforeach()
foreach(module_subdir ${MODULES})
    list(APPEND ROBOT2015_ELF_INCLUDES ${MODULES_ROOT_NAME}/${module_subdir})
endforeach()

# firmware source files
file(GLOB_RECURSE robot2015_SRC "*.cpp")

# Include the arm toolchain for gcc
include(${ARM_TOOLCHAIN_FILE})
rj_mbed_env()

rj_add_mbed_executable(robot2015 ${robot2015_SRC} ${asm_SRC})
target_link_libraries(robot2015 common2015)
target_include_directories(robot2015 PUBLIC ${ROBOT2015_ELF_INCLUDES})
