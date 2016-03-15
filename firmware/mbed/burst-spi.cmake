# BurstSPI sends SPI data without reading it back, allowing higher speeds
# than the regular SPI library. This is mainly useful at high frequencies
# and large payloads. With a small number of bytes the setting up and finishing
# time will remove any advantage.

include(./mbed_executable.cmake)

# the source files for the library
set(BURSTSPI_SRC BurstSPI_LPC_X.cpp)

rj_mbed_env()
rj_add_external_mbed_library(
    NAME burstspi
    HG_REPO       https://developer.mbed.org/users/Sissors/code/BurstSPI
    HG_TAG              13:bc069279eb37
    SRCS ${BURSTSPI_SRC}
    INCLUDE_DIRS ./
)

# add the external project's path/src info into the accessory library lists
set(MBED_ASSEC_LIBS_INCLUDES    ${MBED_ASSEC_LIBS_INCLUDES} ${burstspi_INCLUDES})
