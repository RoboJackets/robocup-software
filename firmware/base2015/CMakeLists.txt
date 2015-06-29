# this file came from https://github.com/uwearzt/mbed-cmake
# and was modified by Justin Buchanan
#
# The end result of this file is the 'base2015' target
# building this target ouputs an MBED-runnable file at run/base2015.bin, which can be copied onto the MBED

# firmware source files
file(GLOB_RECURSE base2015_SRC
    "*.cpp"
)

add_executable(base2015_elf ${base2015_SRC})
add_dependencies(base2015_elf pre-build)
target_link_libraries(base2015_elf common2015)

# the final product is the .bin file, not the elf one.  We hide this away in the build dir
set_target_properties(base2015_elf PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})

# only build base station firmware if specifically instructed
set_target_properties(base2015_elf PROPERTIES EXCLUDE_FROM_ALL TRUE)

# custom target for creating a .bin file from an elf binary
add_custom_target(base2015
    arm-none-eabi-objcopy -O binary base2015_elf base2015.bin
    COMMAND cp ${CMAKE_CURRENT_BINARY_DIR}/base2015.bin ${PROJECT_SOURCE_DIR}/run/
    DEPENDS base2015_elf
    COMMENT "objcopying to make MBED-compatible executable"
)
set_directory_properties(PROPERTIES ADDITIONAL_MAKE_CLEAN_FILES ${PROJECT_SOURCE_DIR}/run/base2015.bin)

add_custom_target(base2015-prog
    arm-none-eabi-objcopy -O binary base2015_elf base2015.bin
    COMMAND ${PROJECT_SOURCE_DIR}/util/base2015-prog.sh ${PROJECT_SOURCE_DIR}/run/base2015.bin
    DEPENDS base2015
    COMMENT "Copying the base2015 firmware into a single mbed"
)
