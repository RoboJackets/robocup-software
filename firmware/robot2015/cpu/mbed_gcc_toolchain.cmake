# this file tells CMake how to build our robot firmware for the mbed
# this is loosely based on directions here: http://developer.mbed.org/cookbook/mbed-cmake

include(CMakeForceCompiler)

# tells CMake the destination OS
# 'Generic' means an embedded system with no OS
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_VERSION 1)

# set our compilers
cmake_force_c_compiler(arm-none-eabi-gcc GNU)
cmake_force_cxx_compiler(arm-none-eabi-g++ GNU)

# note: the common flags were copied from the Makefile that the mbed online compiler exported for gcc
set(COMMON_FLAGS "-mcpu=cortex-m3 -mthumb -c -g -fno-common -fmessage-length=0 -Wall -fno-exceptions -ffunction-sections -fdata-sections -fomit-frame-pointer -fno-rtti -MMD -MP")
set(CMAKE_CXX_FLAGS "${COMMON_FLAGS} -std=gnu++98")
set(CMAKE_C_FLAGS "${COMMON_FLAGS} -std=gnu99")
set(CMAKE_EXE_LINKER_FLAGS "-mcpu=cortex-m3 -mthumb -Wl,--gc-sections --specs=nano.specs -u _printf_float -u _scanf_float")
