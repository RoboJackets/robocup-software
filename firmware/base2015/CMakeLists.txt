# this file came from https://github.com/uwearzt/mbed-cmake
# and was modified by Justin Buchanan
#
# The end result of this file is the 'robot2015' target
# building this target ouputs an MBED-runnable file at run/robot2015.bin, which can be copied onto the MBED

include_directories(../../common2015/drivers ../../common2015/drivers/CC1101)
include_directories(../../common2015/modules ../../common2015/modules/CommLink modules/CommModule)
include_directories(../../common2015/utils ../../common2015/utils/FirmwareCheck ../../common2015/utils/Logger ../../common2015/utils/RTOS_Management ../../common2015/utils/Watchdog)

# set the arm toolchain
set(CMAKE_TOOLCHAIN_FILE ../mbed/arm_toolchain.cmake)
include(../mbed/arm_toolchain.cmake)

# suppress -rdynamic flag
set(CMAKE_SHARED_LIBRARY_LINK_C_FLAGS "")
set(CMAKE_SHARED_LIBRARY_LINK_CXX_FLAGS "")

# set the target processor and include settings for cmake
set(MBED_TARGET LPC1768)
#set(MBED_TARGET LPC11U24)
#set(MBED_TARGET RBLAB_NRF51822)
set(PREBUILD_DEPS)

include(../mbed/arm_mbed.cmake)

# firmware source files
file(GLOB_RECURSE base2015_SRC
    "*.cpp"
)

add_executable(base2015_elf ${base2015_SRC} ${CMAKE_BINARY_DIR}/common/git_version.cpp ${MBED_OBJECTS})
add_dependencies(base2015_elf pre-build)
target_link_libraries(base2015_elf ${MBED_LIBS})

# the final product is the .bin file, not the elf one.  We hide this away in the build dir
set_target_properties(base2015_elf PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})

# only build robot firmware if specifically instructed
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
    COMMENT "Copies the files created by base2015 into a single mbed"
)


# See common/CMakeLists for more info on how this works
add_custom_command(
    OUTPUT ${CMAKE_BINARY_DIR}/common/git_version.cpp
    DEPENDS git_version
)
