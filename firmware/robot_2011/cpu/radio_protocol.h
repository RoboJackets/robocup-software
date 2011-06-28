#pragma once

#include <stdint.h>

// Speeds given in the most recent forward packet
extern int wheel_command[4];	// -511 to 511
extern int dribble_command;		// -511 to 511
extern int kick_command;		// 0 to 255

extern uint32_t rx_lost_time;

int handle_forward_packet(void);