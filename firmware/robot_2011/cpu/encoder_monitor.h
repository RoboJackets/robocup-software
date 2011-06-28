#pragma once

#include <stdint.h>

// Bits 0-3 correspond to the drive motors
extern uint8_t encoder_faults;

void encoder_monitor(void);
