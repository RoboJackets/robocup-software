#ifndef _ADC_H_
#define _ADC_H_

#include <stdint.h>

void adc_init(void);
uint16_t adc_read(int ch);

#endif // _ADC_H_
