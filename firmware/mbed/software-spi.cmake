# Software SPI allows non-standard SPI pins to be used for interfacing with SPI devices

message(STATUS "downloading source files for the SWSPI library")

ExternalProject_Add(swspi_library
    HG_REPOSITORY       https://developer.mbed.org/users/davervw/code/SWSPI
    HG_TAG              0:6a500a08c7fd
    CONFIGURE_COMMAND   ""
    BUILD_COMMAND       ""
    INSTALL_COMMAND     ""
    UPDATE_COMMAND      ""
)
set_target_properties(swspi_library PROPERTIES EXCLUDE_FROM_ALL TRUE)

# the directory to include for linking in with the common2015 library
ExternalProject_Get_Property(swspi_library SOURCE_DIR)

# the source files that will be added to common2015
set(SWSPI_SRC ${SOURCE_DIR}/SWSPI.cpp)

# add the external project's path/src info into the accessory library lists
set(MBED_ASSEC_LIBS_INCLUDES    ${MBED_ASSEC_LIBS_INCLUDES} ${SOURCE_DIR}   )
set(MBED_ASSEC_LIBS_SRCS        ${MBED_ASSEC_LIBS_SRCS}     ${SWSPI_SRC}    )
set(MBED_ASSEC_LIBS_DEPENDS     ${MBED_ASSEC_LIBS_DEPENDS}  swspi_library   )
