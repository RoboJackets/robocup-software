# The MCP23017 is a Digital I/O expander chip that we use to get more I/O pins
# This CMake script pulls down an MBED library for interfacing with it

include(./mbed_executable.cmake)

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

# add the external project's path/src info into the accessory library lists
set(modserial_INCLUDES ${SOURCE_DIR} ${SOURCE_DIR}/Device)
set(MBED_ASSEC_LIBS_INCLUDES ${MBED_ASSEC_LIBS_INCLUDES} ${modserial_INCLUDES})

set(modserial_SRC
    ${SOURCE_DIR}/MODSERIAL.cpp
    ${SOURCE_DIR}/INIT.cpp
    ${SOURCE_DIR}/PUTC.cpp
    ${SOURCE_DIR}/GETC.cpp
    ${SOURCE_DIR}/FLUSH.cpp
    ${SOURCE_DIR}/RESIZE.cpp
    ${SOURCE_DIR}/ISR_RX.cpp
    ${SOURCE_DIR}/ISR_TX.cpp
    ${SOURCE_DIR}/MODSERIAL_IRQ_INFO.cpp
    ${SOURCE_DIR}/Device/MODSERIAL_LPC1768.cpp
    ${SOURCE_DIR}/Device/MODSERIAL_LPC11U24.cpp
    ${SOURCE_DIR}/Device/MODSERIAL_KL05Z.cpp
    ${SOURCE_DIR}/Device/MODSERIAL_KL25Z.cpp
    ${SOURCE_DIR}/Device/MODSERIAL_KSDK.cpp
)

# Specify that each source file depends on the external project
foreach(src ${modserial_SRC})
    add_custom_command(
        OUTPUT ${src}
        DEPENDS modserial_library
    )
endforeach()

# TDOO(justin): remove
include(${ARM_TOOLCHAIN_FILE})
set(CMAKE_CXX_FLAGS         ${MBED_CMAKE_CXX_FLAGS}         )
set(CMAKE_C_FLAGS           ${MBED_CMAKE_C_FLAGS}           )
set(CMAKE_EXE_LINKER_FLAGS  ${MBED_CMAKE_EXE_LINKER_FLAGS}  )

rj_add_mbed_library(modserial ${modserial_SRC})
target_include_directories(modserial PUBLIC ${modserial_INCLUDES})
target_include_directories(modserial PUBLIC ${MBED_INCLUDE_DIR})
add_dependencies(modserial mbed_libraries)

set(MBED_ASSEC_LIBS_DEPENDS     ${MBED_ASSEC_LIBS_DEPENDS}  modserial)
