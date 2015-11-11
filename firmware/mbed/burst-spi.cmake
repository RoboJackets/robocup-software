# BurstSPI sends SPI data without reading it back, allowing higher speeds
# than the regular SPI library. This is mainly useful at high frequencies
# and large payloads. With a small number of bytes the setting up and finishing
# time will remove any advantage.

message(STATUS "downloading source files for the BurstSPI library")

ExternalProject_Add(burstspi_library
    HG_REPOSITORY       https://developer.mbed.org/users/Sissors/code/BurstSPI
    HG_TAG              13:bc069279eb37
    CONFIGURE_COMMAND   ""
    BUILD_COMMAND       ""
    INSTALL_COMMAND     ""
    UPDATE_COMMAND      ""
)
set_target_properties(burstspi_library PROPERTIES EXCLUDE_FROM_ALL TRUE)

# the directory to include for linking in with the common2015 library
ExternalProject_Get_Property(burstspi_library SOURCE_DIR)

# the source files that will be added to common2015
file(GLOB BURSTSPI_SRC "${SOURCE_DIR}/*.cpp")
# set(BURSTSPI_SRC ${SOURCE_DIR}/BurstSPI_KL25Z.cpp)
# list(APPEND BURSTSPI_SRC ${SOURCE_DIR}/BurstSPI_KL46Z.cpp)
# list(APPEND BURSTSPI_SRC ${SOURCE_DIR}/BurstSPI_LPC_1549.cpp)
# list(APPEND BURSTSPI_SRC ${SOURCE_DIR}/BurstSPI_LPC_X.cpp)
# list(APPEND BURSTSPI_SRC ${SOURCE_DIR}/BurstSPI_NUCLEO_L152RE.cpp)
# list(APPEND BURSTSPI_SRC ${SOURCE_DIR}/BurstSPI_STM32F4.cpp)
# list(APPEND BURSTSPI_SRC ${SOURCE_DIR}/BurstSPI_Unsupported.cpp)

# add the external project's path/src info into the accessory library lists
set(MBED_ASSEC_LIBS_INCLUDES    ${MBED_ASSEC_LIBS_INCLUDES} ${SOURCE_DIR}       )
set(MBED_ASSEC_LIBS_SRCS        ${MBED_ASSEC_LIBS_SRCS}     ${BURSTSPI_SRC}     )
set(MBED_ASSEC_LIBS_DEPENDS     ${MBED_ASSEC_LIBS_DEPENDS}  burstspi_library    )
