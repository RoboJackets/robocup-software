#pragma once

#include <stdint.h>

extern uint16_t adc[8];

void adc_init(void);
void adc_update(void);