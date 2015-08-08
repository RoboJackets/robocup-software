
# set the avr toolchain
set(CMAKE_TOOLCHAIN_FILE attiny_toolchain.cmake)
include(attiny_toolchain.cmake)

# F_CPU specifies the frequency (in Hz) of the cpu clock so that the delay functions work appropriately
# the ATTiny13 has an internal 9.6MHz clock with the prescaler, don't scale back
# to keep timing for neopixel
# The -O2 turns on optimizations, which are required to use the delay_*() functions
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O2 -mmcu=attiny13 -DF_CPU=9600000")
# set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O2 -mmcu=attiny441 -DF_CPU=9600000")

file(GLOB kicker_SRC
    "*.c"
)

# compile executable for kicker firmware
add_executable(kicker2015_elf ${kicker_SRC})
# only build kicker firmware if specifically instructed
set_target_properties(kicker2015_elf PROPERTIES EXCLUDE_FROM_ALL TRUE)
# the final product is the .bin file, not the elf one.  We hide this away in the build dir
set_target_properties(kicker2015_elf PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})


# custom target to convert kicker.elf to the kickerFW binary file and place it in the 'run' directory
add_custom_target(kicker2015
    # the -j options tell objcopy what sections to include in the output
    # the -O ihex specifies what output format to use, which is the hex format needed for flashing the avr
    COMMAND avr-objcopy -j .text -j .data -O binary ${CMAKE_CURRENT_BINARY_DIR}/kicker2015_elf ${PROJECT_SOURCE_DIR}/run/kickerFW
    DEPENDS kicker2015_elf
    COMMENT "objcopying to convert kicker.elf to kickerFW"
)
set_directory_properties(PROPERTIES ADDITIONAL_MAKE_CLEAN_FILES ${PROJECT_SOURCE_DIR}/run/kickerFW)

# add command to copy the kicker firmware onto the mbed
add_custom_target(kicker2015-prog
    COMMAND ${MBED_COPY_SCRIPT} ${PROJECT_SOURCE_DIR}/run/kckr-fw
    DEPENDS kicker2015
    COMMENT "Copies kickerFW onto the mbed"
)
