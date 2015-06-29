# This sets up MBED all MBED libraries, then sets up the robot and base station
# firmware, which depend on the MBED libs


# set the target processor and include settings for cmake
set(MBED_TARGET LPC1768)
set(PREBUILD_DEPS)

# set the arm toolchain
set(CMAKE_TOOLCHAIN_FILE arm_toolchain.cmake)
include(mbed/arm_toolchain.cmake)

include(mbed/arm_mbed.cmake)

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

# don't get the external project unless it's required
set_target_properties(mbed_lib_build_tools PROPERTIES EXCLUDE_FROM_ALL TRUE)

# pre-build target will pull and compile mbed libs
# this is added as a dependency for standard builds, forcing the python script to
# run at the correct time
add_custom_target(pre-build
    COMMAND python2 ${PY_TOOLS_DIR}/workspace_tools/build.py --mcu=${PY_MCU_OPT} --tool=${PY_TOOLCHAIN_OPT} ${PY_LIBS}
    DEPENDS ${PREBUILD_DEPS})
set_target_properties(pre-build PROPERTIES EXCLUDE_FROM_ALL TRUE)

# suppress -rdynamic flag
set(CMAKE_SHARED_LIBRARY_LINK_C_FLAGS "")
set(CMAKE_SHARED_LIBRARY_LINK_CXX_FLAGS "")


# header files
include_directories("common2015/drivers" )
include_directories("common2015/drivers/CC1101")
include_directories("common2015/modules" )
include_directories("common2015/modules/CommLink")
include_directories("common2015/modules/CommModule")
include_directories("common2015/utils")
include_directories("common2015/utils/FirmwareCheck")
include_directories("common2015/utils/Logger")
include_directories("common2015/utils/RTOS_Management")
include_directories("common2015/utils/Watchdog")

# build robot and base station firmware and the library that they depend on
add_subdirectory(common2015)
add_subdirectory(base2015)
add_subdirectory(robot2015)
