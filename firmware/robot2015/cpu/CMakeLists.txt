# this file came from https://github.com/uwearzt/mbed-cmake
# and was modified by Justin Buchanan
#
# The end result of this file is the 'robot2015' target
# building this target ouputs an MBED-runnable file at run/robot2015.bin, which can be copied onto the MBED

# set the arm toolchain
set(CMAKE_TOOLCHAIN_FILE arm_toolchain.cmake)
include(arm_toolchain.cmake)


# suppress -rdynamic flag
set(CMAKE_SHARED_LIBRARY_LINK_C_FLAGS "")
set(CMAKE_SHARED_LIBRARY_LINK_CXX_FLAGS "")

include(ExternalProject)

ExternalProject_Add(mbed_library
    HG_REPOSITORY https://developer.mbed.org/users/mbed_official/code/mbed/
    HG_TAG default
    CONFIGURE_COMMAND ""
    BUILD_COMMAND ""
    INSTALL_COMMAND ""
    UPDATE_COMMAND ""
)

# don't get the external project unless it's required
set_target_properties(mbed_library PROPERTIES EXCLUDE_FROM_ALL TRUE)


# set the target processor and include settings for cmake
set(MBED_TARGET LPC1768)
#set(MBED_TARGET LPC11U24)
#set(MBED_TARGET RBLAB_NRF51822)

# ------------------------------------------------------------------------------
# will add the include dirs and link the lib
#set(USE_RTOS true)
#set(USE_NET true)
#set(USE_USB true)
#set(USE_DSP true)


include(arm_mbed.cmake)


# firmware source files
file(GLOB robot2015_SRC
    "*.cpp"
)

add_executable(robot2015_elf ${robot2015_SRC} ${MBED_OBJECTS})
target_link_libraries(robot2015_elf ${MBED_LIBS})

# the final product is the .bin file, not the elf one.  We hide this away in the build dir
set_target_properties(robot2015_elf PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})

# only build robot firmware if specifically instructed
set_target_properties(robot2015_elf PROPERTIES EXCLUDE_FROM_ALL TRUE)


# tell CMake that the obj files all come from the ExternalProject
# otherwise it'll complain that the files can't be found
foreach(mbed_obj ${MBED_OBJECTS})
    add_custom_command(
        OUTPUT ${mbed_obj}
        DEPENDS mbed_library
        COMMAND "")
endforeach()


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
    COMMENT "Copies the files created by robot2015 into a single mbed"
)

