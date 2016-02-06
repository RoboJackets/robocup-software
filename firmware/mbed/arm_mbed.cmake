# =============================================================================
# Mbed setup for interfacing into the official mbed SDK's build system.
# =============================================================================

include(MbedUtil)

CMAKE_MINIMUM_REQUIRED(VERSION 3.0.0)

# ------------------------------------------------------------------------------
# turn build options on/off with there cmake options
RJ_ADD_OP(WITH_MBED_RTOS        "Include mbed RTOS support"                 ON )
RJ_ADD_OP(WITH_MBED_RPC         "Include mbed RPC support"                  OFF)
RJ_ADD_OP(WITH_MBED_USB         "Include mbed USB support"                  OFF)
RJ_ADD_OP(WITH_MBED_USB_HOST    "Include mbed USB Host support"             OFF)
RJ_ADD_OP(WITH_MBED_DSP         "Include mbed DSP support"                  OFF)
RJ_ADD_OP(WITH_MBED_ETH         "Include mbed ethernet/networking support"  OFF)
RJ_ADD_OP(WITH_MBED_FATFS       "Include mbed FAT filesystem support"       OFF)
RJ_ADD_OP(WITH_MBED_UBLOX       "Include mbed UBLOX support"                OFF)
RJ_ADD_OP(WITH_MBED_CPPCHECK    "Include mbed cppcheck support"             OFF)

# ------------------------------------------------------------------------------
# set the toolchain that the mbed SDK build system will use for compiling the
# official mbed libraries
set(MBED_TOOLCHAIN "GCC_ARM")

# ------------------------------------------------------------------------------
# set the platform that we are compiling for
mbed_set_platform(MBED_TARGET TARGET "LPC1768")
# mbed_set_platform(MBED_TARGET TARGET "NRF51_MICROBIT_BOOT")

# ------------------------------------------------------------------------------
# check to make sure the toolchain is supported for the target
list (FIND MBED_TARGET_TOOLCHAINS ${MBED_TOOLCHAIN} _index)
if (NOT ${_index} GREATER -1)
    message(FATAL_ERROR "unsupported toolchain: ${MBED_TOOLCHAIN}")
endif()

# ------------------------------------------------------------------------------
# git checkout and build location of mbed libraries
set(MBED_REPO_DIR "${CMAKE_CURRENT_BINARY_DIR}/mbed_libraries-prefix/src/mbed_libraries")

# ------------------------------------------------------------------------------
# set the path to the core set of mbed SDK's build output files
set(MBED_PATH "${MBED_REPO_DIR}/build/mbed")

# ------------------------------------------------------------------------------
# set the target platform & generate upppercase/lowercase versions of the name
set(MBED_PLATFORM           LPC1768)
string(TOUPPER ${MBED_PLATFORM} MBED_PLATFORM_UPPERC)
string(TOLOWER ${MBED_PLATFORM} MBED_PLATFORM_LOWERC)

# ------------------------------------------------------------------------------
# setup the target platoform's vendor
string(REGEX MATCH "LPC" MBED_NXP ${MBED_PLATFORM_UPPERC})
string(REGEX MATCH "NRF" MBED_NORDIC ${MBED_PLATFORM_UPPERC})

if(MBED_NXP)
    set(MBED_TARGET_VENDOR "NXP")
elseif( MBED_NORDIC )
    set(MBED_TARGET_VENDOR "NORDIC")
else()
    message(FATAL_ERROR "Unknown or unset mbed platform")
endif()

# ------------------------------------------------------------------------------
# set the target platform's family
set(MBED_TARGET_FAMILY "")


# ------------------------------------------------------------------------------
set(MBED_TARGET_ISA "M3")
set(MBED_TARGET_STARTUP "startup_LPC17xx.o")
set(MBED_TARGET_SYSTEM "system_LPC17xx.o")
set(MBED_TARGET_FAMILY "LPC176X")

# set(MBED_TARGET_ISA "M0")
# set(MBED_TARGET_STARTUP "startup_LPC11xx.o")
# set(MBED_TARGET_SYSTEM "system_LPC11Uxx.o")
# set(MBED_TARGET_FAMILY "LPC11UXX")

# set(MBED_TARGET_ISA "M0")
# set(MBED_TARGET_STARTUP "startup_NRF51822.o")
# set(MBED_TARGET_SYSTEM "system_nrf51822.o")
# set(MBED_TARGET_FAMILY "MCU_NRF51822")

# ------------------------------------------------------------------------------
# compiler settings
SET(COMMON_FLAGS ${COMMON_FLAGS} "-Wall -Wextra -Wno-unused-parameter -Wno-missing-field-initializers -fno-exceptions -fno-builtin -MMD -fno-delete-null-pointer-checks")
SET(COMMON_FLAGS ${COMMON_FLAGS} "-mcpu=${MBED_TARGET_CORE} -O2 -mthumb -fno-exceptions -msoft-float -ffunction-sections -fdata-sections -g -fno-common -fmessage-length=0")
set(COMMON_FLAGS ${COMMON_FLAGS} "${CMAKE_CXX_FLAGS}")

SET(MBED_DEFINES ${MBED_DEFINES} "-DTARGET_${MBED_PLATFORM_UPPERC}")
SET(MBED_DEFINES ${MBED_DEFINES} "-DTARGET_${MBED_TARGET_ISA}")
SET(MBED_DEFINES ${MBED_DEFINES} "-DTARGET_${MBED_TARGET_VENDOR}")
SET(MBED_DEFINES ${MBED_DEFINES} "-DTOOLCHAIN_${MBED_TOOLCHAIN}")
# SET(MBED_DEFINES ${MBED_DEFINES} "-DTOOLCHAIN_GCC")

SET(MBED_CMAKE_CXX_FLAGS    "${COMMON_FLAGS} ${MBED_DEFINES}")
SET(MBED_CMAKE_C_FLAGS      "${COMMON_FLAGS} ${MBED_DEFINES} -std=gnu99")

# ------------------------------------------------------------------------------
# libraries for mbed
set(MBED_LIBS mbed stdc++ supc++ m gcc g c nosys rdimon)

# ------------------------------------------------------------------------------
# linker settings
set(MBED_CMAKE_EXE_LINKER_FLAGS "-Wl,--gc-sections -Wl,--wrap,main --specs=nano.specs  -u _printf_float -u _scanf_float")
set(MBED_CMAKE_EXE_LINKER_FLAGS "${MBED_CMAKE_EXE_LINKER_FLAGS} -T '${MBED_PATH}/TARGET_${MBED_PLATFORM_UPPERC}/${MBED_TOOLCHAIN}/${MBED_LINK_TARGET}.ld' -static")

# ------------------------------------------------------------------------------
# mbed include directories for all targets
set(MBED_INC_DIRS ${MBED_INC_DIRS} "${MBED_PATH}/")
set(MBED_INC_DIRS ${MBED_INC_DIRS} "${MBED_PATH}/TARGET_${MBED_PLATFORM_UPPERC}/")
set(MBED_INC_DIRS ${MBED_INC_DIRS} "${MBED_PATH}/TARGET_${MBED_PLATFORM_UPPERC}/${MBED_TOOLCHAIN}")
set(MBED_INC_DIRS ${MBED_INC_DIRS} "${MBED_PATH}/TARGET_${MBED_PLATFORM_UPPERC}/TARGET_${MBED_TARGET_VENDOR}/TARGET_${MBED_TARGET_FAMILY}/")
set(MBED_INC_DIRS ${MBED_INC_DIRS} "${MBED_PATH}/TARGET_${MBED_PLATFORM_UPPERC}/TARGET_${MBED_TARGET_VENDOR}/TARGET_${MBED_TARGET_FAMILY}/TARGET_${MBED_TARGET_PROGEN_UPPERC}")

# ------------------------------------------------------------------------------
# compile with support for the networking library
if(${WITH_MBED_ETH})
    mbed_add_incs_eth(MBED_INC_DIRS LIB_ROOT ${MBED_REPO_DIR} VENDOR ${MBED_TARGET_VENDOR})
    set(MBED_OPT_LIBS       ${MBED_OPT_LIBS}    --eth)
    set(MBED_TARGET_OBJS    ${MBED_TARGET_OBJS} "${MBED_REPO_DIR}/eth/TARGET_${MBED_PLATFORM_UPPERC}/TOOLCHAIN_${MBED_TOOLCHAIN}/libeth.a")
    set(WITH_MBED_RTOS ON)
    set(MBED_CMAKE_CXX_FLAGS ${MBED_CMAKE_CXX_FLAGS} "-Wno-literal-suffix")
endif()

# ------------------------------------------------------------------------------
# compile with support for the RTOS library
if(${WITH_MBED_RTOS})
    mbed_add_incs_rtos(MBED_INC_DIRS LIB_ROOT ${MBED_REPO_DIR} ARCH "CORTEX_M")
    set(MBED_OPT_LIBS       ${MBED_OPT_LIBS}    --rtos)
    set(MBED_TARGET_OBJS    ${MBED_TARGET_OBJS} "${MBED_REPO_DIR}/TARGET_${MBED_PLATFORM_UPPERC}/TOOLCHAIN_${MBED_TOOLCHAIN}/librtos.a")
    set(MBED_TARGET_OBJS    ${MBED_TARGET_OBJS} "${MBED_REPO_DIR}/TARGET_${MBED_PLATFORM_UPPERC}/TOOLCHAIN_${MBED_TOOLCHAIN}/librtx.a")
endif()

# ------------------------------------------------------------------------------
# compile with support for the USB library
if(${WITH_MBED_USB})
    mbed_add_incs_usb(MBED_INC_DIRS LIB_ROOT ${MBED_REPO_DIR} BUILD_HOST)
    set(MBED_OPT_LIBS       ${MBED_OPT_LIBS}    --usb --usb_host)
    set(MBED_TARGET_OBJS    ${MBED_TARGET_OBJS} "${MBED_REPO_DIR}/build/usb/TARGET_${MBED_PLATFORM_UPPERC}/TOOLCHAIN_${MBED_TOOLCHAIN}/libUSBDevice.a")
    set(MBED_TARGET_OBJS    ${MBED_TARGET_OBJS} "${MBED_REPO_DIR}/build/usb_host/TARGET_${MBED_PLATFORM_UPPERC}/TOOLCHAIN_${MBED_TOOLCHAIN}/libUSBHost.a")
endif()

# ------------------------------------------------------------------------------
# compile with support for the DSP library
# if(${WITH_MBED_DSP})
#     mbed_add_incs_dsp(MBED_INC_DIRS LIB_ROOT ${MBED_REPO_DIR})
#     set(MBED_OPT_LIBS       ${MBED_OPT_LIBS}    --dsp)
#     set(MBED_TARGET_OBJS    ${MBED_TARGET_OBJS} "${MBED_REPO_DIR}/build/dsp/TARGET_${MBED_PLATFORM_UPPERC}/TOOLCHAIN_${MBED_TOOLCHAIN}/libcmsis_dsp.a"
#     set(MBED_TARGET_OBJS    ${MBED_TARGET_OBJS} "${MBED_REPO_DIR}/build/dsp/TARGET_${MBED_PLATFORM_UPPERC}/TOOLCHAIN_${MBED_TOOLCHAIN}/libdsp.a")
# endif()

# ------------------------------------------------------------------------------
# compile with support for the RPC library
if(${WITH_MBED_RPC})
    mbed_add_incs_rpc(MBED_INC_DIRS LIB_ROOT ${MBED_REPO_DIR})
    set(MBED_OPT_LIBS       ${MBED_OPT_LIBS}    --rpc)
    set(MBED_TARGET_OBJS    ${MBED_TARGET_OBJS} "${MBED_REPO_DIR}/build/rpc/TARGET_${MBED_PLATFORM_UPPERC}/TOOLCHAIN_${MBED_TOOLCHAIN}/librpc.a")
endif()

# ------------------------------------------------------------------------------
# compile with support for the FAT filesystem library
if(${WITH_MBED_FATFS})
    mbed_add_incs_fatfs(MBED_INC_DIRS LIB_ROOT ${MBED_REPO_DIR})
    set(MBED_OPT_LIBS       ${MBED_OPT_LIBS}    --fat)
endif()

# ------------------------------------------------------------------------------
# compile with support for the UBLOX library
if(${WITH_MBED_UBLOX})
    mbed_add_incs_ublox(MBED_INC_DIRS LIB_ROOT ${MBED_REPO_DIR})
    set(MBED_OPT_LIBS       ${MBED_OPT_LIBS}    --ublox)
endif()

# ------------------------------------------------------------------------------
# compile with support for the cppcheck library tests
if(${WITH_MBED_CPPCHECK})
    mbed_add_incs_cppcheck(MBED_INC_DIRS LIB_ROOT ${MBED_REPO_DIR})
    set(MBED_OPT_LIBS       ${MBED_OPT_LIBS}    --cppcheck)
endif()

# ------------------------------------------------------------------------------
# include the mbed paths and link the toolchain where this file is included in another file
set(MBED_LINK_DIRS "${MBED_PATH}/TARGET_${MBED_PLATFORM_UPPERC}/${MBED_TOOLCHAIN}")
link_directories(${MBED_LINK_DIRS})
include_directories(${MBED_PATH})

# ------------------------------------------------------------------------------
# official MBED libraries
include(ExternalProject)
ExternalProject_Add(mbed_libraries
    URL                 ${PROJECT_SOURCE_DIR}/external/mbed_lib_build_tools
    CONFIGURE_COMMAND   ""
    BUILD_COMMAND       python2 ${MBED_REPO_DIR}/workspace_tools/build.py
                                --mcu=${MBED_PLATFORM_UPPERC}
                                --tool=${MBED_TOOLCHAIN}
                                ${MBED_OPT_LIBS}
    INSTALL_COMMAND     ""
    UPDATE_COMMAND      ""
)
set_target_properties(mbed_libraries PROPERTIES EXCLUDE_FROM_ALL TRUE)

# ------------------------------------------------------------------------------
# setup precompiled mbed files which will be needed for all projects
set(MBED_TARGET_OBJS
    "${MBED_TARGET_OBJS}"
    "${MBED_PATH}/TARGET_${MBED_PLATFORM_UPPERC}/${MBED_TOOLCHAIN}/${MBED_TARGET_STARTUP}"
    "${MBED_PATH}/TARGET_${MBED_PLATFORM_UPPERC}/${MBED_TOOLCHAIN}/${MBED_TARGET_SYSTEM}"
    "${MBED_PATH}/TARGET_${MBED_PLATFORM_UPPERC}/${MBED_TOOLCHAIN}/cmsis_nvic.o"
    "${MBED_PATH}/TARGET_${MBED_PLATFORM_UPPERC}/${MBED_TOOLCHAIN}/retarget.o"
    "${MBED_PATH}/TARGET_${MBED_PLATFORM_UPPERC}/${MBED_TOOLCHAIN}/board.o"
)

# tell CMake that the obj files all come from the ExternalProject
# otherwise it'll complain that the files can't be found
# ------------------------------------------------------------------------------
foreach(mbed_obj ${MBED_TARGET_OBJS})
    add_custom_command(
        OUTPUT      ${mbed_obj}
        DEPENDS     mbed_libraries
        COMMAND     ""
    )
endforeach()

show_vars()
