# ------------------------------------------------------------------------------
# Copyright by Uwe Arzt mailto:mail@uwe-arzt.de, https://uwe-arzt.de
# under BSD License, see https://uwe-arzt.de/bsd-license/
# ------------------------------------------------------------------------------
CMAKE_MINIMUM_REQUIRED(VERSION 2.8.10)

# ------------------------------------------------------------------------------
# git checkout and build location of mbed libraries
set(MBED_PATH ${CMAKE_CURRENT_BINARY_DIR}/mbed_library-prefix/src/mbed_library)

# ------------------------------------------------------------------------------
# setup processor settings add aditional boards here
#  LPC1768, LPC11U24, NRF51822, K64F

# TARGET -> has to be set in CMakeLists.txt
#
# MBED_VENDOR -> CPU Manufacturer
#
message(STATUS "building for ${MBED_TARGET}")
# the settings for mbed is really messed up ;)
if(MBED_TARGET MATCHES "LPC1768")
  set(MBED_VENDOR "NXP")
  set(MBED_FAMILY "LPC176X")
  set(MBED_CPU "MBED_LPC1768")
  set(MBED_CORE "cortex-m3")
  set(MBED_INSTRUCTIONSET "M3")

  set(MBED_STARTUP "startup_LPC17xx.o")
  set(MBED_SYSTEM "system_LPC17xx.o")
  set(MBED_LINK_TARGET ${MBED_TARGET})

elseif(MBED_TARGET MATCHES "LPC11U24")
  set(MBED_VENDOR "NXP")
  set(MBED_FAMILY "LPC11UXX")
  set(MBED_CPU "LPC11U24_401")
  set(MBED_CORE "cortex-m0")
  set(MBED_INSTRUCTIONSET "M0")

  set(MBED_STARTUP "startup_LPC11xx.o")
  set(MBED_SYSTEM "system_LPC11Uxx.o")
  set(MBED_LINK_TARGET ${MBED_TARGET})

elseif(MBED_TARGET MATCHES "RBLAB_NRF51822")
  set(MBED_VENDOR "NORDIC")
  set(MBED_FAMILY "MCU_NRF51822")
  set(MBED_CPU "RBLAB_NRF51822")
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

SET(CMAKE_CXX_FLAGS "${COMMON_FLAGS} ${MBED_DEFINES} -std=gnu++0x")
SET(CMAKE_C_FLAGS "${COMMON_FLAGS} ${MBED_DEFINES} -std=gnu99")


# ------------------------------------------------------------------------------
# setup precompiled mbed files which will be needed for all projects
set(MBED_OBJECTS
  ${MBED_PATH}/TARGET_${MBED_TARGET}/${TOOLCHAIN}/${MBED_STARTUP}
  ${MBED_PATH}/TARGET_${MBED_TARGET}/${TOOLCHAIN}/${MBED_SYSTEM}
  ${MBED_PATH}/TARGET_${MBED_TARGET}/${TOOLCHAIN}/cmsis_nvic.o
  ${MBED_PATH}/TARGET_${MBED_TARGET}/${TOOLCHAIN}/retarget.o
  ${MBED_PATH}/TARGET_${MBED_TARGET}/${TOOLCHAIN}/board.o
)

# ------------------------------------------------------------------------------
# libraries for mbed
set(MBED_LIBS mbed stdc++ supc++ m gcc g c nosys rdimon)

# ------------------------------------------------------------------------------
# linker settings
set(CMAKE_EXE_LINKER_FLAGS "-Wl,--gc-sections -Wl,--wrap,main --specs=nano.specs  -u _printf_float -u _scanf_float")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} \"-T${MBED_PATH}/TARGET_${MBED_TARGET}/${TOOLCHAIN}/${MBED_LINK_TARGET}.ld\" -static")

# ------------------------------------------------------------------------------
# mbed
include_directories("${MBED_PATH}/")
include_directories("${MBED_PATH}/TARGET_${MBED_TARGET}/")
include_directories("${MBED_PATH}/TARGET_${MBED_TARGET}/${TOOLCHAIN}")
include_directories("${MBED_PATH}/TARGET_${MBED_TARGET}/TARGET_${MBED_VENDOR}/TARGET_${MBED_FAMILY}/")
include_directories("${MBED_PATH}/TARGET_${MBED_TARGET}/TARGET_${MBED_VENDOR}/TARGET_${MBED_FAMILY}/TARGET_${MBED_CPU}")

link_directories("${MBED_PATH}/TARGET_${MBED_TARGET}/${TOOLCHAIN}")

# add networking
if(${USE_NET} STREQUAL "true")
  include_directories("${MBED_PATH}/net/eth/")
  include_directories("${MBED_PATH}/net/eth/EthernetInterface")
  include_directories("${MBED_PATH}/net/eth/Socket")
  include_directories("${MBED_PATH}/net/eth/TARGET_${MBED_TARGET}/")
  include_directories("${MBED_PATH}/net/eth/TARGET_${MBED_TARGET}/${TOOLCHAIN}")

  include_directories("${MBED_PATH}/net/eth/lwip")
  include_directories("${MBED_PATH}/net/eth/lwip/include")
  include_directories("${MBED_PATH}/net/eth/lwip/include/ipv4")
  include_directories("${MBED_PATH}/net/eth/lwip-sys")
  include_directories("${MBED_PATH}/net/eth/lwip-eth/arch/TARGET_${MBED_VENDOR}")

  link_directories("${MBED_PATH}/net/eth/TARGET_${MBED_TARGET}/${TOOLCHAIN}")
  set(MBED_LIBS ${MBED_LIBS} eth)

  # supress lwip warnings with 0x11
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-literal-suffix")

  set(USE_RTOS true)
endif()

# add rtos
if(${USE_RTOS} STREQUAL "true")
  include_directories("${MBED_PATH}/rtos/")
  include_directories("${MBED_PATH}/rtos/TARGET_${MBED_TARGET}/")
  include_directories("${MBED_PATH}/rtos/TARGET_${MBED_TARGET}/${TOOLCHAIN}")

  link_directories("${MBED_PATH}/rtos/TARGET_${MBED_TARGET}/${TOOLCHAIN}")
  set(MBED_LIBS ${MBED_LIBS} rtos rtx)
endif()

# add usb
if(${USE_USB} STREQUAL "true")
  include_directories("${MBED_PATH}/USBDevice/")
  include_directories("${MBED_PATH}/USBDevice/TARGET_${MBED_TARGET}/")
  include_directories("${MBED_PATH}/USBDevice/TARGET_${MBED_TARGET}/${TOOLCHAIN}")

  link_directories("${MBED_PATH}/usb/TARGET_${MBED_TARGET}/${TOOLCHAIN}")
  set(MBED_LIBS ${MBED_LIBS} USBDevice)
endif()

# add dsp
if(${USE_DSP} STREQUAL "true")
  include_directories("${MBED_PATH}/dsp/")
  include_directories("${MBED_PATH}/dsp/TARGET_${MBED_TARGET}/")
  include_directories("${MBED_PATH}/dsp/TARGET_${MBED_TARGET}/${TOOLCHAIN}")

  link_directories("${MBED_PATH}/dsp/TARGET_${MBED_TARGET}/${TOOLCHAIN}")
  set(MBED_LIBS ${MBED_LIBS} cmsis_dsp dsp)
endif()

# print all include directories
get_property(dirs DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)
message(STATUS "Include Directories")
foreach(dir ${dirs})
  message(STATUS "  ${dir}")
endforeach()
