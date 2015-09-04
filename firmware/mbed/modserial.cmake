# The MCP23017 is a Digital I/O expander chip that we use to get more I/O pins
# This CMake script pulls down an MBED library for interfacing with it

message(STATUS "downloading source files for the MODSERIAL library")

ExternalProject_Add(modserial_library
    HG_REPOSITORY       https://developer.mbed.org/users/AjK/code/MODSERIAL
    HG_TAG              25:ae0408ebdd68
    CONFIGURE_COMMAND   ""
    BUILD_COMMAND       ""
    INSTALL_COMMAND     ""
    UPDATE_COMMAND      ""
)
set_target_properties(modserial_library PROPERTIES EXCLUDE_FROM_ALL TRUE)

# the directory to include for linking in with the common2015 library
ExternalProject_Get_Property(modserial_library SOURCE_DIR)

# the source files that will be added to common2015
set(MODSERIAL_SRC ${SOURCE_DIR}/MODSERIAL.cpp)
list(APPEND MODSERIAL_SRC ${SOURCE_DIR}/INIT.cpp)
list(APPEND MODSERIAL_SRC ${SOURCE_DIR}/PUTC.cpp)
list(APPEND MODSERIAL_SRC ${SOURCE_DIR}/GETC.cpp)
list(APPEND MODSERIAL_SRC ${SOURCE_DIR}/FLUSH.cpp)
list(APPEND MODSERIAL_SRC ${SOURCE_DIR}/RESIZE.cpp)
list(APPEND MODSERIAL_SRC ${SOURCE_DIR}/ISR_RX.cpp)
list(APPEND MODSERIAL_SRC ${SOURCE_DIR}/ISR_TX.cpp)
list(APPEND MODSERIAL_SRC ${SOURCE_DIR}/MODSERIAL_IRQ_INFO.cpp)


# add the external project's path/src info into the accessory library lists
set(MBED_ASSEC_LIBS_INCLUDES    ${MBED_ASSEC_LIBS_INCLUDES} ${SOURCE_DIR}       )
set(MBED_ASSEC_LIBS_SRCS        ${MBED_ASSEC_LIBS_SRCS}     ${MODSERIAL_SRC}     )
set(MBED_ASSEC_LIBS_DEPENDS     ${MBED_ASSEC_LIBS_DEPENDS}  modserial_library    )
