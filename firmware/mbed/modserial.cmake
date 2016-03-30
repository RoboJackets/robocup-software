# The MCP23017 is a Digital I/O expander chip that we use to get more I/O pins
# This CMake script pulls down an MBED library for interfacing with it

ExternalProject_Add(modserial_library
    HG_REPOSITORY       https://developer.mbed.org/users/Sissors/code/MODSERIAL
    HG_TAG              41:d8422efe4761
    CONFIGURE_COMMAND   ""
    BUILD_COMMAND       ""
    INSTALL_COMMAND     ""
    UPDATE_COMMAND      ""
)
set_target_properties(modserial_library PROPERTIES EXCLUDE_FROM_ALL TRUE)

# the directory to include for linking in with the common2015 library
ExternalProject_Get_Property(modserial_library SOURCE_DIR)

# the source files that will be added to common2015
set(        MODSERIAL_SRC ${SOURCE_DIR}/MODSERIAL.cpp)
list(APPEND MODSERIAL_SRC ${SOURCE_DIR}/INIT.cpp)
list(APPEND MODSERIAL_SRC ${SOURCE_DIR}/PUTC.cpp)
list(APPEND MODSERIAL_SRC ${SOURCE_DIR}/GETC.cpp)
list(APPEND MODSERIAL_SRC ${SOURCE_DIR}/FLUSH.cpp)
list(APPEND MODSERIAL_SRC ${SOURCE_DIR}/RESIZE.cpp)
list(APPEND MODSERIAL_SRC ${SOURCE_DIR}/ISR_RX.cpp)
list(APPEND MODSERIAL_SRC ${SOURCE_DIR}/ISR_TX.cpp)
list(APPEND MODSERIAL_SRC ${SOURCE_DIR}/MODSERIAL_IRQ_INFO.cpp)

list(APPEND MODSERIAL_SRC ${SOURCE_DIR}/Device/MODSERIAL_LPC1768.cpp)
list(APPEND MODSERIAL_SRC ${SOURCE_DIR}/Device/MODSERIAL_LPC11U24.cpp)
list(APPEND MODSERIAL_SRC ${SOURCE_DIR}/Device/MODSERIAL_KL05Z.cpp)
list(APPEND MODSERIAL_SRC ${SOURCE_DIR}/Device/MODSERIAL_KL25Z.cpp)
list(APPEND MODSERIAL_SRC ${SOURCE_DIR}/Device/MODSERIAL_KSDK.cpp)

# add the external project's path/src info into the accessory library lists
set(MBED_ASSEC_LIBS_INCLUDES    ${MBED_ASSEC_LIBS_INCLUDES} ${SOURCE_DIR}        ${SOURCE_DIR}/Device)
set(MBED_ASSEC_LIBS_SRCS        ${MBED_ASSEC_LIBS_SRCS}     ${MODSERIAL_SRC}     )
set(MBED_ASSEC_LIBS_DEPENDS     ${MBED_ASSEC_LIBS_DEPENDS}  modserial_library    )
