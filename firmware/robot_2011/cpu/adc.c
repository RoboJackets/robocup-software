#include <board.h>

#include "adc.h"

uint_fast16_t adc[8];

void adc_init()
{
	//FIXME - Justify these numbers
	AT91C_BASE_ADC->ADC_MR = AT91C_ADC_SLEEP | (0x3f << 8) | (4 << 16) | (15 << 24);
	// Use all channels except 3
	AT91C_BASE_ADC->ADC_CHER = 0xf7;
	// Start the first conversion
	AT91C_BASE_ADC->ADC_CR = AT91C_ADC_START;
	// Wait for it to finish
	// The WDT must be reset here, but I'm not sure why.  The time since the
	// first reset above should not have been long enough to overflow the WDT.
	AT91C_BASE_WDTC->WDTC_WDCR = 0xa5000001;
	while ((AT91C_BASE_ADC->ADC_SR & 0xff) != 0xf7);
	// Read results
	adc_update();
}

void adc_update()
{
	// Read ADC results
	for (int i = 0; i < 8; ++i)
	{
		adc[i] = *(&AT91C_BASE_ADC->ADC_CDR0 + i);
	}
	
	// Start a new set of ADC conversions
	AT91C_BASE_ADC->ADC_CR = AT91C_ADC_START;
}
