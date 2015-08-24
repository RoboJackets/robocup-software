# this file came from https://github.com/uwearzt/mbed-cmake
# and was modified by Justin Buchanan
# 
# The end result of this file is the 'robot2015' target
# building this target ouputs an MBED-runnable file at run/robot2015.bin, which can be copied onto the MBED

#if(${TOC})
#    set(USE_OWN_LINKER_SCRIPT ${TOC})
#    set(LINKER_DIR ${CMAKE_CURRENT_LIST_DIR}/scripts/linker)
#    set(LINKER_SCRIPT ${LINKER_DIR}/sections_FLASH.ld)
#endif()
# 
#if(${USE_OWN_LINKER_SCRIPT})
#    list(APPEND ${MODULES} param toc)
#endif()

# set the names of each directory and then set the names of the subdirectories within each one
set(DRIVERS_ROOT_NAME   drivers)
set(MODULES_ROOT_NAME   modules)
set(CONFIG_ROOT_NAME    config)
set(DRIVERS             adc dma ds2411 fpga mcp23017 mem-iap mpu-6050 rtos-i2c ws2811)
set(MODULES             commands controller motors)

# generate a list of directories that we need to include from the values set above
set(ROBOT2015_ELF_INCLUDES ${CONFIG_ROOT_NAME} ${DRIVERS_ROOT_NAME} ${MODULES_ROOT_NAME})
foreach(driver_subdir ${DRIVERS})
    list(APPEND ROBOT2015_ELF_INCLUDES ${DRIVERS_ROOT_NAME}/${driver_subdir})
endforeach()
foreach(module_subdir ${MODULES})
    list(APPEND ROBOT2015_ELF_INCLUDES ${MODULES_ROOT_NAME}/${module_subdir})
endforeach()

# Include the arm toolchain for gcc
set(CMAKE_TOOLCHAIN_FILE ${ARM_TOOLCHAIN_FILE})
include(${ARM_TOOLCHAIN_FILE})

# Set compiler and linker flags
set(CMAKE_CXX_FLAGS ${MBED_CMAKE_CXX_FLAGS})
set(CMAKE_C_FLAGS ${MBED_CMAKE_C_FLAGS})
set(CMAKE_EXE_LINKER_FLAGS ${MBED_CMAKE_EXE_LINKER_FLAGS})

# Find the assembly source files and make sure they're compiled using the C compiler
file(GLOB_RECURSE assm_SRC "*.s")
set_property(SOURCE ${assm_SRC} PROPERTY LANGUAGE C)
# comment out below line to add assembly files to the build
unset(assm_SRC)

# firmware source files
file(GLOB_RECURSE robot2015_SRC "*.cpp")

# Exclude FreeRTOS stuff if it happens to be a directory here...
string(REGEX REPLACE ";?[^;]+/FreeRTOS[V]?[0-9\\.]+/[^;]+;?" "${0}" robot2015_SRC "${robot2015_SRC}")

# add the target for making the elf file
add_executable(robot2015_elf ${robot2015_SRC} ${assm_SRC})

# specify that we depend on the common2015 library and link to it
add_dependencies(robot2015_elf common2015)
target_link_libraries(robot2015_elf common2015)

# include 
target_include_directories(robot2015_elf PUBLIC ${ROBOT2015_ELF_INCLUDES})

# the final product is the .bin file, not the elf one.  We hide this away in the build dir
set_target_properties(robot2015_elf PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})

# only build robot firmware if specifically instructed
set_target_properties(robot2015_elf PROPERTIES EXCLUDE_FROM_ALL TRUE)

# custom target for creating a .bin file from an elf binary
add_custom_target(robot2015
    arm-none-eabi-objcopy -O binary robot2015_elf rj-robot.bin
    COMMAND cp ${CMAKE_CURRENT_BINARY_DIR}/rj-robot.bin ${PROJECT_SOURCE_DIR}/run/
    DEPENDS robot2015_elf
    COMMENT "objcopying to make mbed-compatible executable"
)
set_directory_properties(PROPERTIES ADDITIONAL_MAKE_CLEAN_FILES ${PROJECT_SOURCE_DIR}/run/rj-robot.bin)

add_custom_target(robot2015-prog
    arm-none-eabi-objcopy -O binary robot2015_elf rj-robot.bin
    COMMAND ${MBED_COPY_SCRIPT} ${PROJECT_SOURCE_DIR}/run/rj-robot.bin
    DEPENDS robot2015
    COMMENT "Copying the robot's binary over to the mbed"
)
