/**********************************************
 * NeoStrip.cpp
 *
 * Allen Wild
 * March 2014
 *
 * Controls a strip of Adafruit NeoPixels, addressable RGB LEDs
 * Currently, because of the global nature of the IO register and bitmask variables,
 * it is only possible to use one NeoStrip instance at a time.
 *
 * This library supports only the NXP LPC1768!
 */

#include "neostrip.hpp"

// #include <mbed.h>

// uint32_t neo_bitmask;
// uint32_t neo_fio_reg;
// void neo_out(NeoColor*, int);

NeoStrip::NeoStrip(PinName pin, int N) : N(N)
{
	bright = 0.5;
	Nbytes = N * 3;
	strip = (NeoColor*)malloc(N * sizeof(NeoColor));

	if (strip == NULL) {
		printf("NeoStrip: ERROR unable to malloc strip data");
		N = 0;
	}

	gpio_init_out(&gpio, pin);				// initialize GPIO registers
	neo_fio_reg = (uint32_t)gpio.reg_dir;	// set registers and bitmask for
	neo_bitmask = 1 << ((int)pin & 0x1F);	// the assembly to use
}

void NeoStrip::setBrightness(float bright)
{
	this->bright = bright;
}

void NeoStrip::setPixel(int p, int color)
{
	int red = (color & 0xFF0000) >> 16;
	int green = (color & 0x00FF00) >> 8;
	int blue = (color & 0x0000FF);

	setPixel(p, red, green, blue);
}

void NeoStrip::setPixel(int p, uint8_t red, uint8_t green, uint8_t blue)
{
	// set the given pixel's RGB values
	// the array is indexed modulo N to avoid overflow
	strip[p % N].red = (uint8_t)(red * bright);
	strip[p % N].green = (uint8_t)(green * bright);
	strip[p % N].blue = (uint8_t)(blue * bright);
}

void NeoStrip::setPixels(int p, int n, const int* colors)
{
	int r, g, b;

	for (int i = 0; i < n; i++) {
		r = (colors[i] & 0xFF0000) >> 16;
		g = (colors[i] & 0x00FF00) >> 8;
		b = colors[i] & 0x0000FF;
		setPixel(p + i, r, g, b);
	}
}

void NeoStrip::clear()
{
	for (int i = 0; i < N; i++) {
		strip[i].red = 0;
		strip[i].green = 0;
		strip[i].blue = 0;
	}
}

void NeoStrip::write()
{
	__disable_irq();		// disable interrupts
	neo_out(strip, Nbytes);	// output to the strip
	__enable_irq();			// enable interrupts
	wait_us(50);			// wait 50us for the reset pulse
}
