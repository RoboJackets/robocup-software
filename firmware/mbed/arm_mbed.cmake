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
# ExternalProject_Get_Property(PY_TOOLS_DIR )
set(PY_TOOLS_DIR ${CMAKE_CURRENT_BINARY_DIR}/mbed/mbed_libraries-prefix/src/mbed_libraries)

# Main mbed library
set(MBED_PATH          ${PY_TOOLS_DIR}/build/mbed)

# Networking paths
set(MBED_CELLULAR_PATH  ${PY_TOOLS_DIR}/build/net/cellular)
set(MBED_ETH_PATH       ${PY_TOOLS_DIR}/build/net/eth)
set(MBED_HTTPS_PATH     ${PY_TOOLS_DIR}/build/net/https)
set(MBED_LWIP_PATH      ${PY_TOOLS_DIR}/build/net/lwip)

# RPC path
set(MBED_RPC_PATH      ${PY_TOOLS_DIR}/libraries/rpc)

# RTOS path
set(MBED_RTOS_PATH     ${PY_TOOLS_DIR}/build/rtos)

set(MBED_USB_PATH      ${PY_TOOLS_DIR}/build/usb)
set(MBED_USB_HOST_PATH ${PY_TOOLS_DIR}/build/usb_host)
set(MBED_DSP_PATH      ${PY_TOOLS_DIR}/build/dsp)


# ------------------------------------------------------------------------------
# setup processor settings add aditional boards here
#  LPC1768, LPC11U24, NRF51822, K64F

# TARGET -> has to be set in CMakeLists.txt
# 
# MBED_VENDOR -> CPU Manufacturer
# 
# message(STATUS "MBED target set to ${MBED_TARGET}")
# the settings for mbed is really messed up ;)
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
# compiler settings
SET(COMMON_FLAGS "${COMMON_FLAGS} -Wall -Wextra -Wno-unused-parameter -Wno-missing-field-initializers -fno-exceptions -fno-builtin -MMD -fno-delete-null-pointer-checks")
SET(COMMON_FLAGS "${COMMON_FLAGS} -mcpu=${MBED_CORE} -O2 -mthumb -fno-exceptions -msoft-float -ffunction-sections -fdata-sections -g -fno-common -fmessage-length=0")

SET(MBED_DEFINES "${MBED_DEFINES} -DTARGET_${MBED_TARGET}")
SET(MBED_DEFINES "${MBED_DEFINES} -DTARGET_${MBED_INSTRUCTIONSET}")
SET(MBED_DEFINES "${MBED_DEFINES} -DTARGET_${MBED_VENDOR}")
SET(MBED_DEFINES "${MBED_DEFINES} -DTOOLCHAIN_GCC_ARM")
SET(MBED_DEFINES "${MBED_DEFINES} -DTOOLCHAIN_GCC")

SET(MBED_CMAKE_CXX_FLAGS "${COMMON_FLAGS} ${MBED_DEFINES}")
SET(MBED_CMAKE_C_FLAGS "${COMMON_FLAGS} ${MBED_DEFINES} -std=gnu99")


# ------------------------------------------------------------------------------
# setup precompiled mbed files which will be needed for all projects
# MESSAGE(STATUS "mbed path: ${MBED_PATH}")
set(MBED_OBJECTS
  ${MBED_PATH}/TARGET_${MBED_TARGET}/${MBED_TOOLCHAIN}/${MBED_STARTUP}
  ${MBED_PATH}/TARGET_${MBED_TARGET}/${MBED_TOOLCHAIN}/${MBED_SYSTEM}
  ${MBED_PATH}/TARGET_${MBED_TARGET}/${MBED_TOOLCHAIN}/cmsis_nvic.o
  ${MBED_PATH}/TARGET_${MBED_TARGET}/${MBED_TOOLCHAIN}/retarget.o
  ${MBED_PATH}/TARGET_${MBED_TARGET}/${MBED_TOOLCHAIN}/board.o
)

# ------------------------------------------------------------------------------
# libraries for mbed
set(MBED_LIBS mbed stdc++ supc++ m gcc g c nosys rdimon)

# ------------------------------------------------------------------------------
# linker settings
set(MBED_CMAKE_EXE_LINKER_FLAGS "-Wl,--gc-sections -Wl,--wrap,main --specs=nano.specs  -u _printf_float -u _scanf_float")
set(MBED_CMAKE_EXE_LINKER_FLAGS "${MBED_CMAKE_EXE_LINKER_FLAGS} \"-T${MBED_PATH}/TARGET_${MBED_TARGET}/${MBED_TOOLCHAIN}/${MBED_LINK_TARGET}.ld\" -static")

# ------------------------------------------------------------------------------
# mbed
include_directories("${MBED_PATH}/")
include_directories("${MBED_PATH}/TARGET_${MBED_TARGET}/")
include_directories("${MBED_PATH}/TARGET_${MBED_TARGET}/${MBED_TOOLCHAIN}")
include_directories("${MBED_PATH}/TARGET_${MBED_TARGET}/TARGET_${MBED_VENDOR}/TARGET_${MBED_FAMILY}/")
include_directories("${MBED_PATH}/TARGET_${MBED_TARGET}/TARGET_${MBED_VENDOR}/TARGET_${MBED_FAMILY}/TARGET_${MBED_CPU}")


# add networking
if(${MBED_USE_ETH} STREQUAL "true")
  #net
  include_directories("${MBED_ETH_PATH}")
  include_directories("${MBED_ETH_PATH}/EthernetInterface")
  include_directories("${MBED_ETH_PATH}/Socket")
  include_directories("${MBED_ETH_PATH}/lwip")
  include_directories("${MBED_ETH_PATH}/lwip/include")
  include_directories("${MBED_ETH_PATH}/lwip/include/ipv4")
  include_directories("${MBED_ETH_PATH}/lwip/include/lwip")
  include_directories("${MBED_ETH_PATH}/lwip/include/netif")
  include_directories("${MBED_ETH_PATH}/lwip-sys")
  include_directories("${MBED_ETH_PATH}/lwip-sys/arch")
  include_directories("${MBED_ETH_PATH}/lwip-eth/arch/TARGET_${MBED_VENDOR}")

  #library dir
  set(PY_NET_LIB_DIR ${MBED_ETH_PATH}/TARGET_${MBED_TARGET}/TOOLCHAIN_${PY_TOOLCHAIN_OPT})
  #add static
  set(MBED_LIBS ${MBED_LIBS} libeth.a)
  #add build arg to py script command
  set(PY_LIBS ${PY_LIBS} --eth)

  # supress lwip warnings with 0x11
  set(MBED_CMAKE_CXX_FLAGS "${MBED_CMAKE_CXX_FLAGS} -Wno-literal-suffix")

  #force rtos
  set(MBED_USE_RTOS true)
endif()

# add rtos
if(${MBED_USE_RTOS} STREQUAL "true")
  include_directories("${MBED_RTOS_PATH}")
  include_directories("${MBED_RTOS_PATH}/TARGET_${MBED_CORE_GENERIC}")

  set(PY_RTOS_LIB_DIR ${MBED_RTOS_PATH}/TARGET_${MBED_TARGET}/TOOLCHAIN_${PY_TOOLCHAIN_OPT})
  set(MBED_LIBS ${MBED_LIBS} ${PY_RTOS_LIB_DIR}/librtos.a ${PY_RTOS_LIB_DIR}/librtx.a)
  set(PY_LIBS ${PY_LIBS} --rtos)
endif()

# add usb
if(${MBED_USE_USB} STREQUAL "true")
  #usb
  include_directories("${MBED_USB_PATH}/USBAudio")
  include_directories("${MBED_USB_PATH}/USBDevice")
  include_directories("${MBED_USB_PATH}/USBHID")
  include_directories("${MBED_USB_PATH}/USBMIDI")
  include_directories("${MBED_USB_PATH}/USBMSD")
  include_directories("${MBED_USB_PATH}/USBSerial")

  #usb host
  include_directories("${MBED_USB_HOST_PATH}/USBHost")
  include_directories("${MBED_USB_HOST_PATH}/USBHost3GModule")
  include_directories("${MBED_USB_HOST_PATH}/USBHostHID")
  include_directories("${MBED_USB_HOST_PATH}/USBHostHub")
  include_directories("${MBED_USB_HOST_PATH}/USBHostMIDI")
  include_directories("${MBED_USB_HOST_PATH}/USBHostMSD")
  include_directories("${MBED_USB_HOST_PATH}/USBHostSerial")

  #library dirs
  set(PY_USB_LIB_DIR      ${MBED_USB_PATH}/TARGET_${MBED_TARGET}/TOOLCHAIN_${PY_TOOLCHAIN_OPT})
  set(PY_USB_HOST_LIB_DIR ${MBED_USB_HOST_PATH}/TARGET_${MBED_TARGET}/TOOLCHAIN_${PY_TOOLCHAIN_OPT})
  #add statics
  set(MBED_LIBS ${MBED_LIBS} libUSBDevice.a libUSBHost.a)
  #add build arg to py script command
  set(PY_LIBS ${PY_LIBS} --usb --usb_host)
endif()

# add dsp
if(${MBED_USE_DSP} STREQUAL "true")
  #dsp
  include_directories("${MBED_DSP_PATH}")

  #library dir
  set(PY_DSP_LIB_DIR ${MBED_DSP_PATH}/TARGET_${MBED_TARGET}/TOOLCHAIN_${PY_TOOLCHAIN_OPT})
  #add static
  set(MBED_LIBS ${MBED_LIBS} libcmsis_dsp.a libdsp.a)
  #add build arg to py script command
  set(PY_LIBS ${PY_LIBS} --dsp)
endif()

# include the mbed paths and link the toolchain where this file is included in another file
set(MBED_LINK_DIRS "${MBED_PATH}/TARGET_${MBED_TARGET}/${MBED_TOOLCHAIN}")
link_directories(${MBED_LINK_DIRS})
include_directories(${MBED_PATH})


# set variables to each of the accessory library cmake project files
set(RPC_MBED_LIB        ${CMAKE_CURRENT_LIST_DIR}/mbed-rpc.cmake      )
set(MCP23017_MBED_LIB   ${CMAKE_CURRENT_LIST_DIR}/mcp23017.cmake      )
set(BURSTSPI_MBED_LIB   ${CMAKE_CURRENT_LIST_DIR}/burst-spi.cmake     )
set(SWSPI_MBED_LIB      ${CMAKE_CURRENT_LIST_DIR}/software-spi.cmake  )
set(SWI2C_MBED_LIB      ${CMAKE_CURRENT_LIST_DIR}/software-i2c.cmake  )
set(MODSER_MBED_LIB     ${CMAKE_CURRENT_LIST_DIR}/modserial.cmake     )
set(MODDMA_MBED_LIB     ${CMAKE_CURRENT_LIST_DIR}/moddma.cmake        )
set(PIXARRY_MBED_LIB    ${CMAKE_CURRENT_LIST_DIR}/pixelarray.cmake    )

# create a list of which accessory libraries we want to download and add to the common2015 library
set(MBED_ASSEC_LIBS
  ${RPC_MBED_LIB}
  # ${MCP23017_MBED_LIB}
  ${BURSTSPI_MBED_LIB}
  # ${SWSPI_MBED_LIB}
  ${SWI2C_MBED_LIB}
  ${MODSER_MBED_LIB}
  ${MODDMA_MBED_LIB}
  ${PIXARRY_MBED_LIB}
)
