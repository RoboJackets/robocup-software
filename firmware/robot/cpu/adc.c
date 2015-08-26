#include <board.h>

#include "adc.h"

// Use only channels 4 (ball sense detector) and 5 (battery voltage)
static const int ADC_Use_Channels = 0x30;

uint_fast16_t adc[8];

void adc_init() {
    // FIXME - Justify these numbers
    AT91C_BASE_ADC->ADC_MR =
        AT91C_ADC_SLEEP | (0x3f << 8) | (4 << 16) | (15 << 24);
    AT91C_BASE_ADC->ADC_CHER = ADC_Use_Channels;
    // Start the first conversion
    AT91C_BASE_ADC->ADC_CR = AT91C_ADC_START;
    // Wait for it to finish
    // The WDT must be reset here, but I'm not sure why.  The time since the
    // first reset above should not have been long enough to overflow the WDT.
    AT91C_BASE_WDTC->WDTC_WDCR = 0xa5000001;
    while ((AT91C_BASE_ADC->ADC_SR & 0xff) != ADC_Use_Channels)
        ;
    // Read results
    adc_update();
}

void adc_update() {
    // Read ADC results
    for (int i = 0; i < 8; ++i) {
        adc[i] = *(&AT91C_BASE_ADC->ADC_CDR0 + i);
    }

    // Start a new set of ADC conversions
    AT91C_BASE_ADC->ADC_CR = AT91C_ADC_START;
}
