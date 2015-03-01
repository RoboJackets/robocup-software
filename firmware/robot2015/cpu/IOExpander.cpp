#include "IOExpander.h"
/*
 * Creates a new IO Exapnder object on I2C pins 9 and 10.
 * 0x40 refers to the MCP base address. To have more, 
 * use 0x40, 0x42, 0x44 etc.
 */
MCP23017 *expander = new MCP23017(p9, p10, 0x40);
