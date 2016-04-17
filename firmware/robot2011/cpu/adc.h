#pragma once

#include <stdint.h>

// Most recent ADC conversion results.
// Read from here instead of ADC registers.
extern uint_fast16_t adc[8];

void adc_init(void);
void adc_update(void);