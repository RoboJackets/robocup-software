# this file came from https://github.com/uwearzt/mbed-cmake
# and was modified by Justin Buchanan
#
# The end result of this file is the 'robot2015' target
# building this target ouputs an MBED-runnable file at run/robot2015.bin, which can be copied onto the MBED


# set the arm toolchain
set(CMAKE_TOOLCHAIN_FILE ../../mbed/arm_toolchain.cmake)
include(../../mbed/arm_toolchain.cmake)


# firmware source files
set(robot2015_SRC
    "commands.cpp"
    "console.cpp"
    "ds2411.cpp"
    "main.cpp"
    "radio.cpp"
)

# Generate the LQR lookup table at compile time
include_directories(motion_control)
get_filename_component(OUTPUT_LOOKUP_TABLE_FILE "${CMAKE_CURRENT_BINARY_DIR}/motion_control/Robot2015LqrLookupTableValues.cpp" ABSOLUTE)
add_custom_target(lqr_lookup_table_cpp_file
    $<TARGET_FILE:lqr-lookup-table-gen> ${OUTPUT_LOOKUP_TABLE_FILE}
    COMMENT "Generating LQR lookup table from system model."
    DEPENDS lqr-lookup-table-gen
)
add_custom_command(
    OUTPUT ${OUTPUT_LOOKUP_TABLE_FILE}
    DEPENDS lqr_lookup_table_cpp_file
)

# add the auto-generated lookup table to the sources list
list(APPEND robot2015_SRC ${OUTPUT_LOOKUP_TABLE_FILE})


add_executable(robot2015_elf ${robot2015_SRC})
target_link_libraries(robot2015_elf common2015)

# the final product is the .bin file, not the elf one.  We hide this away in the build dir
set_target_properties(robot2015_elf PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})

# only build robot firmware if specifically instructed
set_target_properties(robot2015_elf PROPERTIES EXCLUDE_FROM_ALL TRUE)

# custom target for creating a .bin file from an elf binary
add_custom_target(robot2015
    arm-none-eabi-objcopy -O binary robot2015_elf robot2015.bin
    COMMAND cp ${CMAKE_CURRENT_BINARY_DIR}/robot2015.bin ${PROJECT_SOURCE_DIR}/run/
    DEPENDS robot2015_elf
    COMMENT "objcopying to make MBED-compatible executable"
)
set_directory_properties(PROPERTIES ADDITIONAL_MAKE_CLEAN_FILES ${PROJECT_SOURCE_DIR}/run/robot2015.bin)

add_custom_target(robot2015-prog
    arm-none-eabi-objcopy -O binary robot2015_elf robot2015.bin
    COMMAND ${PROJECT_SOURCE_DIR}/util/robot2015-prog.sh ${PROJECT_SOURCE_DIR}/run/robot2015.bin
    DEPENDS robot2015
    COMMENT "Copying robot2015 firmware into a single mbed"
)
