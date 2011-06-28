#pragma once

#include <stdint.h>

// Speeds given in the most recent forward packet
extern int_fast8_t wheel_command[4];
extern int_fast8_t dribble_command;
extern uint_fast8_t kick_command;

extern uint32_t rx_lost_time;

int handle_forward_packet(void);