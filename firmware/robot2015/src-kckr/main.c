#include <stdlib.h>

#include "kicker.h"
#include "neopixel.h"

/*
 * initialization
 */
void init(void) {
    // force prescalar to keep frequency at 9.6 MHz
    loop_until_bit_is_set(CLKPR, CLKPCE);
    CLKPR = 0;

    // set data pins I/O
    loop_until_bit_is_set(DDRB, 3);  // set B3, DO

    // create led buffer
    initNeopixelBuffer();
}

// void SPI_SlaveInit(void)
// {
// 	/* Set MISO output, all others input */
// 	DDR_SPI = (1 << DD_MISO);

// 	/* Enable SPI */
// 	SPCR = (1 << SPE);
// }

// char SPI_SlaveReceive(void)
// {
// 	/* Wait for reception complete */
// 	while (!(SPSR & (1 << SPIF))) {};

// 	/* Return Data Register */
// 	return SPDR;
// }

/*
 * main
 */
int main(void) {
    init();

    const uint8_t UPPER_LIMIT = 0xFF;
    const uint8_t LOWER_LIMIT = 0x00;
    uint8_t red = UPPER_LIMIT;
    uint8_t green = LOWER_LIMIT;
    uint8_t blue = LOWER_LIMIT;

    while (1) {
        if (red == UPPER_LIMIT && green != UPPER_LIMIT && blue == LOWER_LIMIT)
            green++;
        else if (red != LOWER_LIMIT && green == UPPER_LIMIT &&
                 blue == LOWER_LIMIT)
            red--;
        else if (green == UPPER_LIMIT && blue != UPPER_LIMIT)
            blue++;
        else if (green != LOWER_LIMIT && blue == UPPER_LIMIT)
            green--;
        else if (red != UPPER_LIMIT && blue == UPPER_LIMIT)
            red++;
        else if (red == UPPER_LIMIT && blue != LOWER_LIMIT)
            blue--;

        setLed(red, green, blue, 1);
        writeNeopixels();
        _delay_ms(1);
    }
}
