# The MCP23017 is a Digital I/O expander chip that we use to get more I/O pins
# This CMake script pulls down an MBED library for interfacing with it

include(./mbed_executable.cmake)

set(modserial_SRC
    MODSERIAL.cpp
    INIT.cpp
    PUTC.cpp
    GETC.cpp
    FLUSH.cpp
    RESIZE.cpp
    ISR_RX.cpp
    ISR_TX.cpp
    MODSERIAL_IRQ_INFO.cpp
    Device/MODSERIAL_LPC1768.cpp
    Device/MODSERIAL_LPC11U24.cpp
    Device/MODSERIAL_KL05Z.cpp
    Device/MODSERIAL_KL25Z.cpp
    Device/MODSERIAL_KSDK.cpp
)

set(modserial_HDRS
    ./
    Device
)

rj_add_external_mbed_library(
    NAME modserial
    HG_REPO https://developer.mbed.org/users/Sissors/code/MODSERIAL
    HG_TAG 41:d8422efe4761
    SRCS ${modserial_SRC}
    INCLUDE_DIRS ${modserial_HDRS}
)
set(MBED_ASSEC_LIBS_DEPENDS ${MBED_ASSEC_LIBS_DEPENDS} modserial)

# add the external project's path/src info into the accessory library lists
set(MBED_ASSEC_LIBS_INCLUDES ${MBED_ASSEC_LIBS_INCLUDES} ${modserial_INCLUDES})

# TDOO(justin): remove
include(${ARM_TOOLCHAIN_FILE})
set(CMAKE_CXX_FLAGS         ${MBED_CMAKE_CXX_FLAGS}         )
set(CMAKE_C_FLAGS           ${MBED_CMAKE_C_FLAGS}           )
set(CMAKE_EXE_LINKER_FLAGS  ${MBED_CMAKE_EXE_LINKER_FLAGS}  )
