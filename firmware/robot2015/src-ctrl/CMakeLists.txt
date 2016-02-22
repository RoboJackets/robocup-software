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
set_property(SOURCE ${asm_SRC} PROPERTY COMPILE_FLAGS "-x assembler-with-cpp")

# set the names of each directory and then set the names of the subdirectories within each one
set(DRIVERS_ROOT_NAME   ${CMAKE_CURRENT_SOURCE_DIR}/drivers)
set(MODULES_ROOT_NAME   ${CMAKE_CURRENT_SOURCE_DIR}/modules)
set(CONFIG_ROOT_NAME    ${CMAKE_CURRENT_SOURCE_DIR}/config)
set(CONFIG_ROOT_NAME    ${CONFIG_ROOT_NAME} PARENT_SCOPE)
set(DRIVERS             buzzer ds2411 fpga mcp23017 mpu-6050 ws2811)
set(MODULES             commands motors)

# generate a list of directories that we need to include from the values set above
set(ROBOT2015_ELF_INCLUDES ${CONFIG_ROOT_NAME} ${DRIVERS_ROOT_NAME} ${MODULES_ROOT_NAME})
foreach(driver_subdir ${DRIVERS})
    list(APPEND ROBOT2015_ELF_INCLUDES ${DRIVERS_ROOT_NAME}/${driver_subdir})
endforeach()
foreach(module_subdir ${MODULES})
    list(APPEND ROBOT2015_ELF_INCLUDES ${MODULES_ROOT_NAME}/${module_subdir})
endforeach()

# Set compiler and linker flags
set(CMAKE_CXX_FLAGS         ${MBED_CMAKE_CXX_FLAGS}         )
set(CMAKE_C_FLAGS           ${MBED_CMAKE_C_FLAGS}           )
set(CMAKE_EXE_LINKER_FLAGS  ${MBED_CMAKE_EXE_LINKER_FLAGS}  )

# firmware source files
file(GLOB_RECURSE robot2015_SRC "*.cpp")

# add the target for making the elf file
add_executable(robot2015_elf ${robot2015_SRC} ${asm_SRC})

# specify that we depend on the common2015 library and link to it
add_dependencies(robot2015_elf common2015)
target_link_libraries(robot2015_elf common2015)

# include directories
target_include_directories(robot2015_elf PUBLIC ${ROBOT2015_ELF_INCLUDES})

# the final product is the .bin file, not the elf one.  We hide this away in the build dir
set_target_properties(robot2015_elf PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})

# only build robot firmware if specifically instructed
set_target_properties(robot2015_elf PROPERTIES EXCLUDE_FROM_ALL TRUE)

# custom target for creating a .bin file from an elf binary
add_custom_target(robot2015
    ${ARM_OBJCOPY} -O binary robot2015_elf rj-robot.bin
    COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_BINARY_DIR}/rj-robot.bin ${PROJECT_SOURCE_DIR}/run/
    DEPENDS robot2015_elf
    COMMENT "objcopying to make mbed-compatible executable"
)
set_directory_properties(PROPERTIES ADDITIONAL_MAKE_CLEAN_FILES ${PROJECT_SOURCE_DIR}/run/rj-robot.bin)

add_custom_target(robot2015-prog
    COMMAND ${MBED_COPY_SCRIPT} ${PROJECT_SOURCE_DIR}/run/rj-robot.bin
    DEPENDS robot2015
    COMMENT "Copying the robot's binary over to the mbed"
)
