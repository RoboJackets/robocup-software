# wrapper around add_executable for adding mbed executables
function(rj_add_mbed_executable name)
    set(binfile ${name}.bin)

    add_executable(${name} ${ARGN})
    _rj_configure_mbed_binary(${name})

    # the final product is the .bin file, not the elf one.  We hide this away in the build dir
    set_target_properties(${name} PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})

    # custom target for creating a .bin file from an elf binary
    add_custom_target(${binfile}
        arm-none-eabi-objcopy -O binary ${name} ${PROJECT_SOURCE_DIR}/run/rj-robot.bin # todo: rename binfile
        DEPENDS ${name}
        COMMENT "objcopying to make mbed-compatible executable"
    )
    set_directory_properties(PROPERTIES ADDITIONAL_MAKE_CLEAN_FILES ${PROJECT_SOURCE_DIR}/run/rj-robot.bin)

    add_custom_target(${name}-prog
        COMMAND ${MBED_COPY_SCRIPT} ${PROJECT_SOURCE_DIR}/run/rj-robot.bin
        DEPENDS ${binfile}
        COMMENT "Copying the robot's binary over to the mbed"
    )
endfunction(rj_add_mbed_executable)


function(rj_add_mbed_library name)
    add_library(${name} ${ARGN})
    _rj_configure_mbed_binary(${name})
endfunction(rj_add_mbed_library)


function(_rj_configure_mbed_binary name)
    # Set compiler and linker flags
    set_target_properties(${name} PROPERTIES
        CMAKE_CXX_FLAGS ${MBED_CMAKE_CXX_FLAGS})
    set_target_properties(${name} PROPERTIES
        CMAKE_C_FLAGS ${MBED_CMAKE_C_FLAGS})
    set_target_properties(${name} PROPERTIES
        CMAKE_EXE_LINKER_FLAGS ${MBED_CMAKE_EXE_LINKER_FLAGS})
    # Include the arm toolchain for gcc
    set_target_properties(${name} PROPERTIES
        CMAKE_TOOLCHAIN_FILE ${ARM_TOOLCHAIN_FILE})
    include(${ARM_TOOLCHAIN_FILE})

    # only build this if specifically instructed
    set_target_properties(${name} PROPERTIES EXCLUDE_FROM_ALL TRUE)
endfunction()
