# set the path to where the ARM & AVR toolchains setup files are at so they can be used for all firmware directories
set(ARM_TOOLCHAIN_FILE ${CMAKE_CURRENT_LIST_DIR}/arm_toolchain.cmake)
set(ARM_TOOLCHAIN_FILE ${ARM_TOOLCHAIN_FILE} PARENT_SCOPE)

# set which additional mbed libraries to grab at the bottom of arm_mbed.cmake
foreach(assec_lib ${MBED_ASSEC_LIBS})
    include(${assec_lib})
endforeach()

set(MBED_ASSEC_LIBS_SRCS        ${MBED_ASSEC_LIBS_SRCS}     PARENT_SCOPE)
set(MBED_ASSEC_LIBS_INCLUDES    ${MBED_ASSEC_LIBS_INCLUDES} PARENT_SCOPE)
set(MBED_ASSEC_LIBS_DEPENDS     ${MBED_ASSEC_LIBS_DEPENDS}  PARENT_SCOPE)

# all official libraries
include(ExternalProject)
ExternalProject_Add(mbed_libraries
    URL                 ${PROJECT_SOURCE_DIR}/external/mbed_lib_build_tools
    CONFIGURE_COMMAND   ""
    BUILD_COMMAND       python2 ${PY_TOOLS_DIR}/workspace_tools/build.py --mcu=${PY_MCU_OPT} --tool=${PY_TOOLCHAIN_OPT} ${PY_LIBS}
    INSTALL_COMMAND     ""
    UPDATE_COMMAND      ""
)
set_target_properties(mbed_libraries PROPERTIES EXCLUDE_FROM_ALL TRUE)

ExternalProject_Get_Property(mbed_libraries SOURCE_DIR)

set(MBED_SRC_PROJ_DIR ${SOURCE_DIR} PARENT_SCOPE)
