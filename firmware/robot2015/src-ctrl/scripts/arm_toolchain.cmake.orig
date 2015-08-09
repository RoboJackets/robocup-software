# ------------------------------------------------------------------------------
# Copyright by Uwe Arzt mailto:mail@uwe-arzt.de, https://uwe-arzt.de
# under BSD License, see https://uwe-arzt.de/bsd-license/
# ------------------------------------------------------------------------------
include(CMakeForceCompiler)
 
#-------------------------------------------------------------------------------
set(CMAKE_SYSTEM_NAME "ARM Cortex mbed")
# set(CMAKE_SYSTEM_VERSION 1)

#AS      = $(GCC_BIN)arm-none-eabi-as
#CC      = $(GCC_BIN)arm-none-eabi-gcc
#CPP     = $(GCC_BIN)arm-none-eabi-g++
#LD      = $(GCC_BIN)arm-none-eabi-gcc
#OBJCOPY = $(GCC_BIN)arm-none-eabi-objcopy
#OBJDUMP = $(GCC_BIN)arm-none-eabi-objdump
#SIZE    = $(GCC_BIN)arm-none-eabi-size 

#-------------------------------------------------------------------------------
# specify the cross compiler, later on we will set the correct path
CMAKE_FORCE_C_COMPILER(arm-none-eabi-gcc GNU)
CMAKE_FORCE_CXX_COMPILER(arm-none-eabi-g++ GNU)

#-------------------------------------------------------------------------------
set(TOOLCHAIN TOOLCHAIN_GCC_ARM)

#-------------------------------------------------------------------------------
# define presets
# mbed official
set(USE_RTOS true)
set(USE_NET false)
set(USE_USB false)
set(USE_DSP false)

# other
set(BUILD_MCP23017 false)
