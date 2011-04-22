#pragma once

#include <stdint.h>

extern uint_fast16_t encoder[4];
extern int_fast16_t encoder_delta[4];

// PWM values sent to the FPGA
extern int_fast8_t wheel_out[4];
extern int_fast8_t dribble_out;

extern uint_fast8_t kick_strength;
extern uint_fast8_t use_chipper;

// Initializes and tests the FPGA.
// This does not force the FPGA to reconfigure, but waits for it to finish.
//
// Returns 0 on failure, 1 on successful configuration, or 2 if the FPGA was already configured.
int fpga_init(void);

void fpga_update(void);
