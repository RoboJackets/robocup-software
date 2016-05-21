/*
 * In order to command the KickerBoard, a command byte must be sent,
 * followed by an argument byte. The command byte will not be executed without
 * the argument, even if that argument is not really needed like in the
 * get_voltage command.
 *
 * If getting a varible, it will be returned on the spi write after the
 * command write and the argument write.
 */

/* Commands */
// Duration Commands
#define KICK_CMD 0x01
#define CHIP_CMD 0x02
// Other Commands
#define SET_CHARGE_CMD 0x03
#define GET_VOLTAGE_CMD 0x04
#define PING_CMD 0x05 // Pings the KickerBoard, the board should return the ACK

/* Arguments */
#define NOP_ARG 0x00 // Used for clarity when passing useless arguments
#define MAX_TIME_ARG 0xFF // Used if we want to wait max time
#define DEBUG_KICK_TIME 0x8
#define DEBUG_CHIP_TIME 0x8
#define ON_ARG 0x01 // Used for setting a wire high
#define OFF_ARG 0x00 // Used for setting a wire low, same as NOP_ARG

/* Other */
#define PING_ACK 0x4C // Arbitrary code used to check if board is working
