# The MCP23017 is a Digital I/O expander chip that we use to get more I/O pins
# This CMake script pulls down an MBED library for interfacing with it

ExternalProject_Add(mcp23017_library
    HG_REPOSITORY       https://developer.mbed.org/users/Nurbol/code/MCP23017
    HG_TAG              0:69c047b34ca6
    CONFIGURE_COMMAND   ""
    BUILD_COMMAND       ""
    INSTALL_COMMAND     ""
    UPDATE_COMMAND      ""
)
set_target_properties(mcp23017_library PROPERTIES EXCLUDE_FROM_ALL TRUE)

# the directory to include for linking in with the common2015 library
ExternalProject_Get_Property(mcp23017_library SOURCE_DIR)

# the source files that will be added to common2015
set(MCP23017_SRC ${SOURCE_DIR}/MCP23017.cpp)

# add the external project's path/src info into the accessory library lists
set(MBED_ASSEC_LIBS_INCLUDES    ${MBED_ASSEC_LIBS_INCLUDES} ${SOURCE_DIR}       )
set(MBED_ASSEC_LIBS_SRCS        ${MBED_ASSEC_LIBS_SRCS}     ${MCP23017_SRC}     )
set(MBED_ASSEC_LIBS_DEPENDS     ${MBED_ASSEC_LIBS_DEPENDS}  mcp23017_library    )
