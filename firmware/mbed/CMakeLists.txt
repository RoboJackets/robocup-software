# set the path to where the ARM & AVR toolchains setup files are at so they can be used for all firmware directories
set(ARM_TOOLCHAIN_FILE ${CMAKE_CURRENT_LIST_DIR}/arm_toolchain.cmake)
set(ARM_TOOLCHAIN_FILE ${ARM_TOOLCHAIN_FILE} PARENT_SCOPE)

# create a list of which accessory libraries we want to download and add to the common2015 library
set(MBED_ASSEC_LIBS
    ${CMAKE_CURRENT_LIST_DIR}/burst-spi.cmake
    # ${CMAKE_CURRENT_LIST_DIR}/software-spi.cmake
    # ${CMAKE_CURRENT_LIST_DIR}/software-i2c.cmake
    ${CMAKE_CURRENT_LIST_DIR}/modserial.cmake
    ${CMAKE_CURRENT_LIST_DIR}/moddma.cmake
    # ${CMAKE_CURRENT_LIST_DIR}/pixelarray.cmake
)

# set which additional mbed libraries to grab at the bottom of arm_mbed.cmake
foreach(assec_lib ${MBED_ASSEC_LIBS})
    include(${assec_lib})
endforeach()

set(MBED_ASSEC_LIBS_SRCS        ${MBED_ASSEC_LIBS_SRCS}     PARENT_SCOPE)
set(MBED_ASSEC_LIBS_INCLUDES    ${MBED_ASSEC_LIBS_INCLUDES} PARENT_SCOPE)
set(MBED_ASSEC_LIBS_DEPENDS     ${MBED_ASSEC_LIBS_DEPENDS}  PARENT_SCOPE)
