
# all official libraries
include(ExternalProject)
ExternalProject_Add(mbed_libraries
    GIT_REPOSITORY ${PROJECT_SOURCE_DIR}/external/mbed_lib_build_tools
    CONFIGURE_COMMAND ""
    BUILD_COMMAND python2 ${PY_TOOLS_DIR}/workspace_tools/build.py --mcu=${PY_MCU_OPT} --tool=${PY_TOOLCHAIN_OPT} ${PY_LIBS}
    INSTALL_COMMAND ""
    UPDATE_COMMAND ""
)

# don't get the external project unless it's required
set_target_properties(mbed_libraries PROPERTIES EXCLUDE_FROM_ALL TRUE)
