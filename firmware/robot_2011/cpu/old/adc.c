#include "lpc2103.h"
#include "adc.h"

void adc_init()
{
	ADCR = 0x00221000;
	
	PINSEL0 |= 3 << 24;		// Battery voltage
	PINSEL1 |= 3 << 18;		// Ball sensor
}

uint16_t adc_read(int ch)
{
	ADCR = 0x01221000 | (1 << ch);
	while (!(ADGDR & (1 << 31)))
		;
	return (ADGDR >> 6) & 0x3ff;
}
