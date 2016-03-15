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

file(GLOB_RECURSE modserial_SRC "${SOURCE_DIR}/*.cpp")

# TDOO(justin): remove
include(${ARM_TOOLCHAIN_FILE})
set(CMAKE_CXX_FLAGS         ${MBED_CMAKE_CXX_FLAGS}         )
set(CMAKE_C_FLAGS           ${MBED_CMAKE_C_FLAGS}           )
set(CMAKE_EXE_LINKER_FLAGS  ${MBED_CMAKE_EXE_LINKER_FLAGS}  )

rj_add_mbed_library(modserial ${modserial_SRC})
add_dependencies(modserial modserial_library)
target_include_directories(modserial PUBLIC ${modserial_INCLUDES})

set(MBED_ASSEC_LIBS_DEPENDS     ${MBED_ASSEC_LIBS_DEPENDS}  modserial)
