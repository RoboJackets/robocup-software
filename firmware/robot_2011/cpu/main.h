#pragma once

#include <stdint.h>

#include "radio.h"

// Speeds given in the most recent forward packet
extern int_fast8_t wheel_command[4];
extern int_fast8_t dribble_command;

extern int_fast8_t last_rssi;
extern uint8_t forward_packet[Forward_Size];

int fpga_init(void);
