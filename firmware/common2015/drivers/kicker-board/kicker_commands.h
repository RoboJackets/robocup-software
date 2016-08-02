#pragma once

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
#define PING_CMD 0x05  // Pings the KickerBoard, the board should return the ACK
// Major debug commands, probably to be removed later
#define GET_BUTTON_STATE_CMD 0x06

/* Arguments */
#define NOP_ARG 0x00       // Used for clarity when passing useless arguments
#define MAX_TIME_ARG 0xFF  // Used if we want to wait max time
#define DB_KICK_TIME 0x08
#define DB_CHIP_TIME 0x08
#define ON_ARG 0x38   // Used for setting a wire high
#define OFF_ARG 0x1A  // Used for setting a wire low, same as NOP_ARG
// GET_BUTTON_STATE_CMD args
#define DB_CHIP_STATE 0x01
#define DB_KICK_STATE 0x02
#define DB_CHARGE_STATE 0x03

/*
 * ACK Codes which should be returned after their corresponding commands
 * for now, we just set them to be the actual command issued for simplicity
 */
#define KICK_ACK KICK_CMD
#define CHIP_ACK CHIP_CMD
#define SET_CHARGE_ACK SET_CHARGE_CMD
#define PING_ACK PING_CMD  // Arbitrary code used to check if board is working
