# The MCP23017 is a Digital I/O expander chip that we use to get more I/O pins
# This CMake script pulls down an MBED library for interfacing with it

ExternalProject_Add(mcp23017
    HG_REPOSITORY https://developer.mbed.org/users/Nurbol/code/MCP23017/
    HG_TAG default
    CONFIGURE_COMMAND ""
    BUILD_COMMAND ""
    INSTALL_COMMAND ""
    UPDATE_COMMAND ""
)

set(MCP23017_DIR ${CMAKE_CURRENT_BINARY_DIR}/mcp23017-prefix/src/mcp23017)

include_directories("${MCP23017_DIR}")
