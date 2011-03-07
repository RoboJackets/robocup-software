#pragma once

#include <stdint.h>

#include "radio.h"

// Most recent ADC conversion results.
// Read from here instead of ADC registers.
extern uint16_t adc[8];

extern uint16_t encoder[4];
extern uint16_t last_encoder[4];
extern int8_t wheel_out[4];
extern int8_t wheel_command[4];
extern int8_t last_rssi;
extern uint8_t forward_packet[Forward_Size];

int fpga_init(void);
