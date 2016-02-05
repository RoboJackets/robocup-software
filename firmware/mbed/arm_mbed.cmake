# ------------------------------------------------------------------------------
# Copyright by Uwe Arzt mailto:mail@uwe-arzt.de, https://uwe-arzt.de
# under BSD License, see https://uwe-arzt.de/bsd-license/
# ------------------------------------------------------------------------------

CMAKE_MINIMUM_REQUIRED(VERSION 3.0.0)
set(MBED_TOOLCHAIN_OPT "GCC_ARM")
set(MBED_OPT_LIBS)

# ------------------------------------------------------------------------------
# git checkout and build location of mbed libraries
set(MBED_SRC_ROOT ${CMAKE_CURRENT_BINARY_DIR}/mbed_libraries-prefix/src/mbed_libraries)
include(${CMAKE_CURRENT_LIST_DIR}/../dump_cmake_vars.cmake)

set(MBED_PLATFORM           LPC1768)
set(MBED_PLATFORM_UPPERC    string(TOUPPER ${MBED_PLATFORM}))
set(MBED_PLATFORM_LOWERC    string(TOUPPER ${MBED_PLATFORM}))

# ------------------------------------------------------------------------------
# setup processor settings add aditional boards here
#  LPC1768, LPC11U24, NRF51822, K64F

string(REGEX MATCH "LPC" MBED_NXP ${MBED_PLATFORM})
string(REGEX MATCH "NRF" MBED_NORDIC ${MBED_PLATFORM})

if( MBED_NXP )
    set(MBED_MCU_VENDOR NXP)
    set(MBED_MCU_FAMILY )
elseif( MBED_NORDIC )
    set(MBED_MCU_VENDOR NORDIC)
    # set the cpu family
    string(SUBSTRING ${MBED_PLATFORM_UPPERC} 0 6 MBED_MCU_FAMILY)
    string(CONCAT MBED_MCU_FAMILY ${MBED_CPU_FAMILY})
else()
    message(FATAL_ERROR "Unknown or unset mbed platform")
endif()

message(STATUS "Using ${MBED_MCU_VENDOR} MCU")
message(STATUS "Using ${MBED_MCU_FAMILY} MCU Family")

# mbed_set_platform()

# if(${MBED_PLATFORM_UPPERC} MATCHES LPC1768)
    # set(MBED_MCU_VENDOR "NXP")
    set(MBED_FAMILY "LPC176X")
    # set(MBED_CPU "MBED_LPC1768")
    set(MBED_CORE "cortex-m3")
    # set(MBED_INSTRUCTIONSET "M3")
    set(MBED_STARTUP "startup_LPC17xx.o")
    set(MBED_SYSTEM "system_LPC17xx.o")
    # set(MBED_LINK_TARGET ${MBED_PLATFORM})

# endif()

# elseif(${MBED_PLATFORM_UPPERC} MATCHES LPC11U24)
    # set(MBED_MCU_VENDOR "NXP")
    set(MBED_FAMILY "LPC11UXX")
    set(MBED_CPU "LPC11U24_401")
    set(MBED_CORE "cortex-m0")
    # set(MBED_INSTRUCTIONSET "M0")
    set(MBED_STARTUP "startup_LPC11xx.o")
    set(MBED_SYSTEM "system_LPC11Uxx.o")
    # set(MBED_LINK_TARGET ${MBED_PLATFORM})

# elseif(${MBED_PLATFORM_UPPERC} MATCHES NRF51822)
    # set(MBED_MCU_VENDOR "NORDIC")
    set(MBED_FAMILY "MCU_NRF51822")
    set(MBED_CPU "RBLAB_NRF51822")
    set(MBED_CORE "cortex-m0")
    # set(MBED_INSTRUCTIONSET "M0")
    set(MBED_STARTUP "startup_NRF51822.o")
    set(MBED_SYSTEM "system_nrf51822.o")
    set(MBED_LINK_TARGET "NRF51822")

# else()
#    message(FATAL_ERROR "No MBED_PLATFORM specified or available. Full stop :(")

# endif()

set(MBED_CPU "MBED_LPC1768")
set(MBED_CPU "RBLAB_NRF51822")
set(MBED_CPU "LPC11U24_401")
set(MBED_LINK_TARGET ${MBED_PLATFORM})
set(MBED_CORE "cortex-m0")
 set(MBED_CORE "cortex-m3")

# ------------------------------------------------------------------------------
# compiler settings
SET(COMMON_FLAGS "${COMMON_FLAGS} -Wall -Wextra -Wno-unused-parameter -Wno-missing-field-initializers -fno-exceptions -fno-builtin -MMD -fno-delete-null-pointer-checks")
SET(COMMON_FLAGS "${COMMON_FLAGS} -mcpu=${MBED_CORE} -O2 -mthumb -fno-exceptions -msoft-float -ffunction-sections -fdata-sections -g -fno-common -fmessage-length=0")
set(COMMON_FLAGS "${COMMON_FLAGS} ${CMAKE_CXX_FLAGS}")

SET(MBED_DEFINES "${MBED_DEFINES} -DTARGET_${MBED_PLATFORM}")
SET(MBED_DEFINES "${MBED_DEFINES} -DTARGET_${MBED_INSTRUCTIONSET}")
SET(MBED_DEFINES "${MBED_DEFINES} -DTARGET_${MBED_MCU_VENDOR}")
SET(MBED_DEFINES "${MBED_DEFINES} -DTOOLCHAIN_GCC_ARM")
SET(MBED_DEFINES "${MBED_DEFINES} -DTOOLCHAIN_GCC")

SET(MBED_CMAKE_CXX_FLAGS "${COMMON_FLAGS} ${MBED_DEFINES}")
SET(MBED_CMAKE_C_FLAGS "${COMMON_FLAGS} ${MBED_DEFINES} -std=gnu99")


# Main mbed library
set(MBED_PATH ${MBED_SRC_ROOT}/build/mbed)


# ------------------------------------------------------------------------------
# libraries for mbed
set(MBED_LIBS mbed stdc++ supc++ m gcc g c nosys rdimon)

# ------------------------------------------------------------------------------
# linker settings
set(MBED_CMAKE_EXE_LINKER_FLAGS "-Wl,--gc-sections -Wl,--wrap,main --specs=nano.specs  -u _printf_float -u _scanf_float")
set(MBED_CMAKE_EXE_LINKER_FLAGS "${MBED_CMAKE_EXE_LINKER_FLAGS} -T '${MBED_PATH}/TARGET_${MBED_PLATFORM}/${MBED_TOOLCHAIN}/${MBED_LINK_TARGET}.ld' -static")

# ------------------------------------------------------------------------------
# mbed
include_directories("${MBED_PATH}/")
include_directories("${MBED_PATH}/TARGET_${MBED_PLATFORM}/")
include_directories("${MBED_PATH}/TARGET_${MBED_PLATFORM}/${MBED_TOOLCHAIN}")
include_directories("${MBED_PATH}/TARGET_${MBED_PLATFORM}/TARGET_${MBED_MCU_VENDOR}/TARGET_${MBED_FAMILY}/")
include_directories("${MBED_PATH}/TARGET_${MBED_PLATFORM}/TARGET_${MBED_MCU_VENDOR}/TARGET_${MBED_FAMILY}/TARGET_${MBED_CPU}")

set(MBED_INC_DIRS "")

# add networking
if(${MBED_USE_ETH})
    mbed_add_incs_eth(MBED_INC_DIRS ${MBED_SRC_ROOT} ${MBED_MCU_VENDOR})
    set(MBED_NET_LIB_DIR ${MBED_SRC_ROOT}/build/net/eth/TARGET_${MBED_PLATFORM}/TOOLCHAIN_${MBED_TOOLCHAIN_OPT})
    set(MBED_LIBS ${MBED_LIBS} ${MBED_NET_LIB_DIR}/libeth.a)
    set(MBED_OPT_LIBS ${MBED_OPT_LIBS} --eth)
    set(MBED_CMAKE_CXX_FLAGS "${MBED_CMAKE_CXX_FLAGS} -Wno-literal-suffix")
    set(MBED_USE_RTOS true)
endif()

# add rtos
if(${MBED_USE_RTOS})
    mbed_add_incs_rtos(MBED_INC_DIRS ${MBED_SRC_ROOT})
    set(MBED_RTOS_LIB_DIR ${MBED_SRC_ROOT}/TARGET_${MBED_PLATFORM}/TOOLCHAIN_${MBED_TOOLCHAIN_OPT})
    set(MBED_LIBS ${MBED_LIBS} ${MBED_RTOS_LIB_DIR}/librtos.a ${MBED_RTOS_LIB_DIR}/librtx.a)
    set(MBED_OPT_LIBS ${MBED_OPT_LIBS} --rtos)
endif()

# add usb
if(${MBED_USE_USB})
    mbed_add_incs_usb(MBED_INC_DIRS ${MBED_SRC_ROOT})
    set(MBED_USB_LIB_DIR      ${MBED_SRC_ROOT}/build/usb/TARGET_${MBED_PLATFORM}/TOOLCHAIN_${MBED_TOOLCHAIN_OPT})
    set(MBED_USB_HOST_LIB_DIR ${MBED_SRC_ROOT}/build/usb_host/TARGET_${MBED_PLATFORM}/TOOLCHAIN_${MBED_TOOLCHAIN_OPT})
    set(MBED_LIBS ${MBED_LIBS} ${MBED_USB_LIB_DIR}/libUSBDevice.a ${MBED_USB_HOST_LIB_DIR}/libUSBHost.a)
    set(MBED_OPT_LIBS ${MBED_OPT_LIBS} --usb --usb_host)
endif()

# add dsp
if(${MBED_USE_DSP})
    mbed_add_incs_dsp(MBED_INC_DIRS ${MBED_SRC_ROOT})
    set(MBED_DSP_LIB_DIR ${MBED_SRC_ROOT}/build/dsp/TARGET_${MBED_PLATFORM}/TOOLCHAIN_${MBED_TOOLCHAIN_OPT})
    set(MBED_LIBS ${MBED_LIBS} ${MBED_DSP_LIB_DIR}/libcmsis_dsp.a ${MBED_DSP_LIB_DIR}/libdsp.a)
    set(MBED_OPT_LIBS ${MBED_OPT_LIBS} --dsp)
endif()

# rpc
if(${MBED_USE_RPC})
    mbed_add_incs_rpc(MBED_INC_DIRS ${MBED_SRC_ROOT})
    set(MBED_LIBS ${MBED_LIBS} ${MBED_SRC_ROOT}/build/rpc/TARGET_${MBED_PLATFORM}/TOOLCHAIN_${MBED_TOOLCHAIN_OPT}/librpc.a)
    set(MBED_OPT_LIBS ${MBED_OPT_LIBS} --rpc)
endif()

# include the mbed paths and link the toolchain where this file is included in another file
set(MBED_LINK_DIRS "${MBED_PATH}/TARGET_${MBED_PLATFORM}/${MBED_TOOLCHAIN}")
link_directories(${MBED_LINK_DIRS})
include_directories(${MBED_PATH})
include_directories(${MBED_INC_DIRS})
message(STATUS ${MBED_INC_DIRS})


# official MBED libraries
include(ExternalProject)
ExternalProject_Add(mbed_libraries
    URL                 ${PROJECT_SOURCE_DIR}/external/mbed_lib_build_tools
    CONFIGURE_COMMAND   ""
    BUILD_COMMAND       python2 ${MBED_SRC_ROOT}/workspace_tools/build.py
                                --mcu=${MBED_PLATFORM}
                                --tool=${MBED_TOOLCHAIN_OPT}
                                ${MBED_OPT_LIBS}
    INSTALL_COMMAND     ""
    UPDATE_COMMAND      ""
)
set_target_properties(mbed_libraries PROPERTIES EXCLUDE_FROM_ALL TRUE)


# ------------------------------------------------------------------------------
# setup precompiled mbed files which will be needed for all projects
set(MBED_OBJECTS
    ${MBED_PATH}/TARGET_${MBED_PLATFORM}/${MBED_TOOLCHAIN}/${MBED_STARTUP}
    ${MBED_PATH}/TARGET_${MBED_PLATFORM}/${MBED_TOOLCHAIN}/${MBED_SYSTEM}
    ${MBED_PATH}/TARGET_${MBED_PLATFORM}/${MBED_TOOLCHAIN}/cmsis_nvic.o
    ${MBED_PATH}/TARGET_${MBED_PLATFORM}/${MBED_TOOLCHAIN}/retarget.o
    ${MBED_PATH}/TARGET_${MBED_PLATFORM}/${MBED_TOOLCHAIN}/board.o
)

# tell CMake that the obj files all come from the ExternalProject
# otherwise it'll complain that the files can't be found
foreach(mbed_obj ${MBED_OBJECTS})
    add_custom_command(
        OUTPUT      ${mbed_obj}
        DEPENDS     mbed_libraries
        COMMAND     ""
    )
endforeach()
