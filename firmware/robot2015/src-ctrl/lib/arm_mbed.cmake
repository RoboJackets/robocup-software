# ------------------------------------------------------------------------------
# Copyright by Uwe Arzt mailto:mail@uwe-arzt.de, https://uwe-arzt.de
# under BSD License, see https://uwe-arzt.de/bsd-license/
# ------------------------------------------------------------------------------
CMAKE_MINIMUM_REQUIRED(VERSION 2.8.10)
set(PY_TOOLCHAIN_OPT "GCC_ARM")
set(PY_MCU_OPT ${MBED_TARGET})
set(PY_LIBS)


# ------------------------------------------------------------------------------
# git checkout and build location of mbed libraries
set(PY_TOOLS_DIR ${CMAKE_CURRENT_BINARY_DIR}/mbed_lib_build_tools-prefix/src/mbed_lib_build_tools)
set(MCP23017_DIR ${CMAKE_CURRENT_BINARY_DIR}/mcp23017-prefix/src/mcp23017)


# Library roots
set(MBED_PATH          ${PY_TOOLS_DIR}/build/mbed)
set(MBED_NET_PATH      ${PY_TOOLS_DIR}/build/net/eth)
set(MBED_RTOS_PATH     ${PY_TOOLS_DIR}/build/rtos)
set(MBED_USB_PATH      ${PY_TOOLS_DIR}/build/usb)
set(MBED_USB_HOST_PATH ${PY_TOOLS_DIR}/build/usb_host)
set(MBED_DSP_PATH      ${PY_TOOLS_DIR}/build/dsp)


# ------------------------------------------------------------------------------
# Setup processor settings add aditional boards here
#  LPC1768, LPC11U24, NRF51822, K64F

# TARGET -> has to be set in CMakeLists.txt
# MBED_VENDOR -> CPU Manufacturer
message(STATUS "Building for ${MBED_TARGET}")


# The settings for mbed is really messed up ;)
if(MBED_TARGET MATCHES "LPC1768")
  set(MBED_VENDOR "NXP")
  set(MBED_FAMILY "LPC176X")
  set(MBED_CPU "MBED_LPC1768")
  set(MBED_CORE "cortex-m3")
  set(MBED_CORE_GENERIC "CORTEX_M")
  set(MBED_INSTRUCTIONSET "M3")

  set(MBED_STARTUP "startup_LPC17xx.o")
  set(MBED_SYSTEM "system_LPC17xx.o")
  set(MBED_LINK_TARGET ${MBED_TARGET})

elseif(MBED_TARGET MATCHES "LPC11U24")
  set(MBED_VENDOR "NXP")
  set(MBED_FAMILY "LPC11UXX")
  set(MBED_CPU "LPC11U24_401")
  set(MBED_CORE_GENERIC "CORTEX_M")
  set(MBED_CORE "cortex-m0")
  set(MBED_INSTRUCTIONSET "M0")

  set(MBED_STARTUP "startup_LPC11xx.o")
  set(MBED_SYSTEM "system_LPC11Uxx.o")
  set(MBED_LINK_TARGET ${MBED_TARGET})

elseif(MBED_TARGET MATCHES "RBLAB_NRF51822")
  set(MBED_VENDOR "NORDIC")
  set(MBED_FAMILY "MCU_NRF51822")
  set(MBED_CPU "RBLAB_NRF51822")
  set(MBED_CORE_GENERIC "CORTEX_M")
  set(MBED_CORE "cortex-m0")
  set(MBED_INSTRUCTIONSET "M0")

  set(MBED_STARTUP "startup_NRF51822.o")
  set(MBED_SYSTEM "system_nrf51822.o")
  set(MBED_LINK_TARGET "NRF51822")

else()
   message(FATAL_ERROR "No MBED_TARGET specified or available. Full stop :(")
endif()


# ------------------------------------------------------------------------------
# Compiler settings
SET(COMMON_FLAGS "${COMMON_FLAGS} -Wall -Wextra -Wno-unused-parameter -Wno-missing-field-initializers -fno-exceptions -fno-builtin -MMD -fno-delete-null-pointer-checks")
SET(COMMON_FLAGS "${COMMON_FLAGS} -mcpu=${MBED_CORE} -O2 -mthumb -fno-exceptions -msoft-float -ffunction-sections -fdata-sections -g -fno-common -fmessage-length=0")

SET(MBED_DEFINES "${MBED_DEFINES} -DTARGET_${MBED_TARGET}")
SET(MBED_DEFINES "${MBED_DEFINES} -DTARGET_${MBED_INSTRUCTIONSET}")
SET(MBED_DEFINES "${MBED_DEFINES} -DTARGET_${MBED_VENDOR}")
SET(MBED_DEFINES "${MBED_DEFINES} -DTOOLCHAIN_GCC_ARM")
SET(MBED_DEFINES "${MBED_DEFINES} -DTOOLCHAIN_GCC")

SET(CMAKE_CXX_FLAGS "${COMMON_FLAGS} ${MBED_DEFINES} -std=gnu++0x")
SET(CMAKE_C_FLAGS "${COMMON_FLAGS} ${MBED_DEFINES} -std=gnu99")


# ------------------------------------------------------------------------------
# Setup precompiled mbed files which will be needed for all projects
set(MBED_OBJECTS
  ${MBED_PATH}/TARGET_${MBED_TARGET}/${TOOLCHAIN}/${MBED_STARTUP}
  ${MBED_PATH}/TARGET_${MBED_TARGET}/${TOOLCHAIN}/${MBED_SYSTEM}
  ${MBED_PATH}/TARGET_${MBED_TARGET}/${TOOLCHAIN}/cmsis_nvic.o
  ${MBED_PATH}/TARGET_${MBED_TARGET}/${TOOLCHAIN}/retarget.o
  ${MBED_PATH}/TARGET_${MBED_TARGET}/${TOOLCHAIN}/board.o
)


# ------------------------------------------------------------------------------
# Libraries for mbed
set(MBED_LIBS mbed stdc++ supc++ m gcc g c nosys rdimon)


# ------------------------------------------------------------------------------
# Linker settings
if(${USE_OWN_LINKER_SCRIPT})
  if(EXISTS ${LINKER_SCRIPT})
    add_definitions(-DLINK_TOC_PARAMS=TRUE)
    include_directories("${LINKER_DIR}/")
    message(STATUS "Using non-default linker script located at ${LINKER_SCRIPT}.")
  endif()
else()
  set(LINKER_SCRIPT ${MBED_PATH}/TARGET_${MBED_TARGET}/${TOOLCHAIN}/${MBED_LINK_TARGET}.ld)
  message(STATUS "Using default linker script located at ${LINKER_SCRIPT}.")
endif()

set(CMAKE_EXE_LINKER_FLAGS "-Wl,--gc-sections -Wl,--wrap,main --specs=nano.specs  -u _printf_float -u _scanf_float")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} \"-T${LINKER_SCRIPT}\" -static")


# ------------------------------------------------------------------------------
# mbed
include_directories("${MBED_PATH}/")
include_directories("${MBED_PATH}/TARGET_${MBED_TARGET}/")
include_directories("${MBED_PATH}/TARGET_${MBED_TARGET}/${TOOLCHAIN}")
include_directories("${MBED_PATH}/TARGET_${MBED_TARGET}/TARGET_${MBED_VENDOR}/TARGET_${MBED_FAMILY}/")
include_directories("${MBED_PATH}/TARGET_${MBED_TARGET}/TARGET_${MBED_VENDOR}/TARGET_${MBED_FAMILY}/TARGET_${MBED_CPU}")

link_directories("${MBED_PATH}/TARGET_${MBED_TARGET}/${TOOLCHAIN}")


# Add Networking
if(${USE_NET} STREQUAL "true")
  # Net
  include_directories("${MBED_NET_PATH}")
  include_directories("${MBED_NET_PATH}/EthernetInterface")
  include_directories("${MBED_NET_PATH}/Socket")
  include_directories("${MBED_NET_PATH}/lwip")
  include_directories("${MBED_NET_PATH}/lwip/include")
  include_directories("${MBED_NET_PATH}/lwip/include/ipv4")
  include_directories("${MBED_NET_PATH}/lwip/include/lwip")
  include_directories("${MBED_NET_PATH}/lwip/include/netif")
  include_directories("${MBED_NET_PATH}/lwip-sys")
  include_directories("${MBED_NET_PATH}/lwip-sys/arch")
  include_directories("${MBED_NET_PATH}/lwip-eth/arch/TARGET_${MBED_VENDOR}")

  # Library dir
  set(PY_NET_LIB_DIR ${MBED_NET_PATH}/TARGET_${MBED_TARGET}/TOOLCHAIN_${PY_TOOLCHAIN_OPT})
  # Add static
  set(MBED_LIBS ${MBED_LIBS} ${PY_NET_LIB_DIR}/libeth.a)
  # Add build arg to py script command
  set(PY_LIBS ${PY_LIBS} --eth)

  # Supress lwip warnings with 0x11
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-literal-suffix")

  # Force rtos
  set(USE_RTOS true)
endif()


# Add RTOS
if(${USE_RTOS} STREQUAL "true")
  include_directories("${MBED_RTOS_PATH}")
  include_directories("${MBED_RTOS_PATH}/TARGET_${MBED_CORE_GENERIC}")

  set(PY_RTOS_LIB_DIR ${MBED_RTOS_PATH}/TARGET_${MBED_TARGET}/TOOLCHAIN_${PY_TOOLCHAIN_OPT})
  set(MBED_LIBS ${MBED_LIBS} ${PY_RTOS_LIB_DIR}/librtos.a ${PY_RTOS_LIB_DIR}/librtx.a)
  set(PY_LIBS ${PY_LIBS} --rtos)
endif()


# Add usb
if(${USE_USB} STREQUAL "true")
  # USB
  include_directories("${MBED_USB_PATH}/USBAudio")
  include_directories("${MBED_USB_PATH}/USBDevice")
  include_directories("${MBED_USB_PATH}/USBHID")
  include_directories("${MBED_USB_PATH}/USBMIDI")
  include_directories("${MBED_USB_PATH}/USBMSD")
  include_directories("${MBED_USB_PATH}/USBSerial")

  # USB Host
  include_directories("${MBED_USB_HOST_PATH}/USBHost")
  include_directories("${MBED_USB_HOST_PATH}/USBHost3GModule")
  include_directories("${MBED_USB_HOST_PATH}/USBHostHID")
  include_directories("${MBED_USB_HOST_PATH}/USBHostHub")
  include_directories("${MBED_USB_HOST_PATH}/USBHostMIDI")
  include_directories("${MBED_USB_HOST_PATH}/USBHostMSD")
  include_directories("${MBED_USB_HOST_PATH}/USBHostSerial")

  # Library dirs
  set(PY_USB_LIB_DIR      ${MBED_USB_PATH}/TARGET_${MBED_TARGET}/TOOLCHAIN_${PY_TOOLCHAIN_OPT})
  set(PY_USB_HOST_LIB_DIR ${MBED_USB_HOST_PATH}/TARGET_${MBED_TARGET}/TOOLCHAIN_${PY_TOOLCHAIN_OPT})
  # Add statics
  set(MBED_LIBS ${MBED_LIBS} ${PY_USB_LIB_DIR}/libUSBDevice.a ${PY_USB_HOST_LIB_DIR}/libUSBHost.a)
  # Add build arg to py script command
  set(PY_LIBS ${PY_LIBS} --usb --usb_host)
endif()


# Add DSP
if(${USE_DSP} STREQUAL "true")
  # DSP
  include_directories("${MBED_DSP_PATH}")

  # Library dir
  set(PY_DSP_LIB_DIR ${MBED_DSP_PATH}/TARGET_${MBED_TARGET}/TOOLCHAIN_${PY_TOOLCHAIN_OPT})
  # Add static
  set(MBED_LIBS ${MBED_LIBS} ${PY_DSP_LIB_DIR}/libcmsis_dsp.a ${PY_DSP_LIB_DIR}/libdsp.a)
  # Add build arg to py script command
  set(PY_LIBS ${PY_LIBS} --dsp)
endif()

if(${BUILD_MCP23017} STREQUAL "true")
  include_directories("${MCP23017_DIR}")
endif()
