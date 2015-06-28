include(arm_toolchain.cmake)

#all official libraries pulled with/by the build tools
include(ExternalProject)
ExternalProject_Add(mbed_lib_build_tools
    GIT_REPOSITORY https://github.com/mbedmicro/mbed.git
    CONFIGURE_COMMAND ""
    BUILD_COMMAND ""
    INSTALL_COMMAND ""
    UPDATE_COMMAND ""
)
set(PREBUILD_DEPS ${PREBUILD_DEPS} mbed_lib_build_tools)

#pull DIO breakout
if(${BUILD_MCP23017} STREQUAL "true")
    ExternalProject_Add(mcp23017
        HG_REPOSITORY https://developer.mbed.org/users/Nurbol/code/MCP23017/
        HG_TAG default
        CONFIGURE_COMMAND ""
        BUILD_COMMAND ""
        INSTALL_COMMAND ""
        UPDATE_COMMAND ""
    )
    set(PREBUILD_DEPS ${PREBUILD_DEPS} mcp23017)
endif()

# don't get the external project unless it's required
set_target_properties(mbed_lib_build_tools PROPERTIES EXCLUDE_FROM_ALL TRUE)

#pre-build target will pull and compile mbed libs
#this is added as a dependency for standard builds, forcing the python script to
#run at the correct time
add_custom_target(pre-build
    COMMAND python2 ${PY_TOOLS_DIR}/workspace_tools/build.py --mcu=${PY_MCU_OPT} --tool=${PY_TOOLCHAIN_OPT} ${PY_LIBS}
    DEPENDS ${PREBUILD_DEPS})
set_target_properties(pre-build PROPERTIES EXCLUDE_FROM_ALL TRUE)

# tell CMake that the obj files all come from the ExternalProject
# otherwise it'll complain that the files can't be found
foreach(mbed_obj ${MBED_OBJECTS})
    add_custom_command(
        OUTPUT ${mbed_obj}
	DEPENDS pre-build
        COMMAND "")
endforeach()


