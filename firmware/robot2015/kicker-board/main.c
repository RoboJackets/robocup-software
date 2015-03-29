#include <stdlib.h>
#include "kicker.h"
#include "neopixel.h"

/*
 * initialization
 */
void init()
{
	//force prescalar to keep frequency at 9.6 MHz
	CLKPR = (1<<CLKPCE);
	CLKPR = 0;

	//set data pins I/O
	DDRB = 1<<3; //set B3, DO

	//create led buffer
	initNeopixelBuffer();
}

/*
 * main
 */
int main(void)
{
	init();
	
	const uint8_t UPPER_LIMIT = 0xFF;
	const uint8_t LOWER_LIMIT = 0x00;
	uint8_t red   = UPPER_LIMIT;
	uint8_t green = LOWER_LIMIT; 
	uint8_t blue  = LOWER_LIMIT;
	for (;;)
	{
		if 	(red   == UPPER_LIMIT && green != UPPER_LIMIT && blue == LOWER_LIMIT)
			green++;
		else if (red   != LOWER_LIMIT && green == UPPER_LIMIT && blue == LOWER_LIMIT)
			red--;	
		else if (green == UPPER_LIMIT && blue  != UPPER_LIMIT)
			blue++;
		else if (green != LOWER_LIMIT && blue  == UPPER_LIMIT)
			green--;
		else if (red   != UPPER_LIMIT && blue  == UPPER_LIMIT)
			red++;
		else if (red   == UPPER_LIMIT && blue  != LOWER_LIMIT)
			blue--;

		setLed(red, green, blue, 1);
		writeNeopixels();
		_delay_ms(1);	
	}
}
