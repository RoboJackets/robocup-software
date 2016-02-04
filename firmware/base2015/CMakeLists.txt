# set the ARM toolchain
set(CMAKE_TOOLCHAIN_FILE ${ARM_TOOLCHAIN_FILE})
include(${ARM_TOOLCHAIN_FILE})

set(CMAKE_C_COMPILER    ${ARM_CC_COMPILER})
set(CMAKE_CXX_COMPILER  ${ARM_CXX_COMPILER})

# Set compiler and linker flags
set(CMAKE_CXX_FLAGS ${MBED_CMAKE_CXX_FLAGS})
set(CMAKE_C_FLAGS ${MBED_CMAKE_C_FLAGS})
set(CMAKE_EXE_LINKER_FLAGS ${MBED_CMAKE_EXE_LINKER_FLAGS})

# firmware source files
set(base2015_SRC "main.cpp")

add_executable(base2015_elf ${base2015_SRC})
target_link_libraries(base2015_elf common2015)

# the final product is the .bin file, not the elf one.  We hide this away in the build dir
set_target_properties(base2015_elf PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})

# only build base station firmware if specifically instructed
set_target_properties(base2015_elf PROPERTIES EXCLUDE_FROM_ALL TRUE)

# custom target for creating a .bin file from an elf binary
add_custom_target(base2015
    arm-none-eabi-objcopy -O binary base2015_elf rj-base.bin
    COMMAND cp ${CMAKE_CURRENT_BINARY_DIR}/rj-base.bin ${PROJECT_SOURCE_DIR}/run/
    DEPENDS base2015_elf
    COMMENT "objcopying to make MBED-compatible executable"
)
set_directory_properties(PROPERTIES ADDITIONAL_MAKE_CLEAN_FILES ${PROJECT_SOURCE_DIR}/run/rj-base.bin)

add_custom_target(base2015-prog
    arm-none-eabi-objcopy -O binary base2015_elf rj-base.bin
    COMMAND ${MBED_COPY_SCRIPT} ${PROJECT_SOURCE_DIR}/run/rj-base.bin
    DEPENDS base2015
    COMMENT "Copying base station's binary over to the mbed"
)
