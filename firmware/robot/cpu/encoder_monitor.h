#pragma once

#include <stdint.h>

// Bits 0-3 correspond to the drive motors
extern uint8_t encoder_faults;

extern int em_err_hall[5], em_err_enc[5], em_err_out[5];

void encoder_monitor(void);
