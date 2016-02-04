
# set the AVR toolchain
set(CMAKE_TOOLCHAIN_FILE attiny_toolchain.cmake)
include(attiny_toolchain.cmake)

set(CMAKE_C_COMPILER    ${AVR_CC_COMPILER})
set(CMAKE_CXX_COMPILER  ${AVR_CXX_COMPILER})

message(STATUS "AVR CC_COMPILER: "  ${AVR_CC_COMPILER})
message(STATUS "AVR_CXX_COMPILER: " ${AVR_CXX_COMPILER})

# F_CPU specifies the frequency (in Hz) of the cpu clock so that the delay functions work appropriately
# the ATTiny13 has an internal 9.6MHz clock with the prescaler, don't scale back
# to keep timing for neopixel
# The -O2 turns on optimizations, which are required to use the delay_*() functions
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O2 -mmcu=attiny44a -DF_CPU=9600000")

# Select the C source files
file(GLOB kicker_SRC "*.c")

# compile executable for kicker firmware
add_executable(kicker2015_elf ${kicker_SRC})

# only build kicker firmware if specifically instructed
set_target_properties(kicker2015_elf PROPERTIES EXCLUDE_FROM_ALL TRUE)

# the final product is the .bin file, not the elf one.  We hide this away in the build dir
set_target_properties(kicker2015_elf PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})

# custom target to convert kicker.elf to the kickerFW binary file and place it in the 'run' directory
add_custom_target(kicker2015
    # the -j options tell objcopy what sections to include in the output
    # the -O ihex specifies what output format to use, which is the hex format needed for flashing the AVR
    COMMAND avr-objcopy -j .text -j .data -O binary ${CMAKE_CURRENT_BINARY_DIR}/kicker2015_elf ${PROJECT_SOURCE_DIR}/run/rj-kickr.nib
    DEPENDS kicker2015_elf
    COMMENT "objcopying to convert kicker.elf to rj-kickr.nib ${CMAKE_CXX_COMPILER} ${CMAKE_C_COMPILER} ${CMAKE_ASM_COMPILER}"
)
set_directory_properties(PROPERTIES ADDITIONAL_MAKE_CLEAN_FILES ${PROJECT_SOURCE_DIR}/run/rj-kickr.nib)

# add command to copy the kicker firmware onto the mbed
# names are limited to 8.3 filename types since we need on-board access to this during runtime
# lookup the limitations of an 8.3 filename before changing the output binary name if that's ever needed
add_custom_target(kicker2015-prog
    COMMAND ${MBED_COPY_SCRIPT} ${PROJECT_SOURCE_DIR}/run/rj-kickr.nib
    DEPENDS kicker2015
    COMMENT "Copying the kicker board's binary over to the mbed"
)
