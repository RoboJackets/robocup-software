#pragma once

#include <stdint.h>

// Bits 0-4 indicate a stall on the corresponding motor
// FIXME - The dribbler has no stall detection.  Need to put hall counters back in the FPGA.
extern uint8_t motor_stall;

extern int stall_counter[5];

void stall_init(void);
void stall_update(void);
