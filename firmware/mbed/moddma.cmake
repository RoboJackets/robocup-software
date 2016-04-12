# MODDMA GPDMA Controller New features: transfer pins to memory buffer
# under periodic timer control and send double buffers to DAC

include(./mbed_executable.cmake)

# Include the arm toolchain for gcc
include(${ARM_TOOLCHAIN_FILE})
rj_mbed_env()

# the source files for the library
set(MODDMA_SRC
    MODDMA.cpp
    INIT.cpp
    SETUP.cpp
    DATALUTS.cpp
)

rj_add_external_mbed_library(
    NAME moddma
    HG_REPO https://developer.mbed.org/users/AjK/code/MODDMA
    HG_TAG 17:97a16bf2ff43
    SRCS ${MODDMA_SRC}
    INCLUDE_DIRS ./
)

# add the external project's include directories
set(MBED_ASSEC_LIBS_INCLUDES ${MBED_ASSEC_LIBS_INCLUDES} ${moddma_INCLUDES})
