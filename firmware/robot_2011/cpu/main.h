#pragma once

#include <stdint.h>

#include "radio.h"

// Speeds given in the most recent forward packet
extern int_fast8_t wheel_command[4];
extern int_fast8_t dribble_command;

extern uint8_t forward_packet[Forward_Size];
extern uint32_t rx_lost_time;

// If nonzero, this is called after every update if the USB console is available
extern void (*debug_update)(void);

int fpga_init(void);
