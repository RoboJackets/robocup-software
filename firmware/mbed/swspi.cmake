
ExternalProject_Add(swspi_library
    HG_REPOSITORY       https://developer.mbed.org/users/davervw/code/SWSPI
    HG_TAG              default
    CONFIGURE_COMMAND   ""
    BUILD_COMMAND       ""
    INSTALL_COMMAND     ""
    UPDATE_COMMAND      ""
)
set_target_properties(swspi_library PROPERTIES EXCLUDE_FROM_ALL TRUE)

set(SWSPI_DIR ${CMAKE_CURRENT_BINARY_DIR}/swspi_library-prefix/src/swspi_library)
set(SWSPI_SRC ${SWSPI_DIR}/SWSPI.cpp)
set(MBED_ASSEC_LIBS_PATH ${MBED_ASSEC_LIBS_PATH} ${SWSPI_DIR})
