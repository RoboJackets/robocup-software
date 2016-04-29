/* Commands */
#define NOP_CMD 0x0
// Duration Commands
#define KICK_CMD 0x1
#define CHIP_CMD 0x2
// Other Commands
#define CHARGE_ON_CMD 0x3
#define CHARGE_OFF_CMD 0x4
#define GET_VOLTAGE_CMD 0x5

#define NOP_ARG 0x0
/*
 * Each byte sent is split into a command and argument (like duration for kicks)
 * Bits 7:4 | Bits 3:0
 *   CMD    |   ARG
 */
// To be used on the ATTINY84A side
#define PARSE_CMD(BYTE) ((BYTE & (0xF << 4)) >> 4)
#define PARSE_ARG(BYTE) (BYTE & 0xF)
// To be used on the MBED side
// These cut CMD and ARG down to 4 bits and ARG is in millis for kick and chip
#define MAKE_BYTE(CMD, ARG) (((CMD & 0xF) << 4) | (ARG & 0xF))
