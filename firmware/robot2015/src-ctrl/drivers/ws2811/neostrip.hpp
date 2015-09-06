/**
 * NeoStrip.h
 *
 * Allen Wild
 * March 2014
 *
 * Library for the control of Adafruit NeoPixel addressable RGB LEDs.
 */

#pragma once

#include <mbed.h>

#ifndef TARGET_LPC1768
#error NeoStrip only supports the NXP LPC1768!
#else

// NeoColor struct definition to hold 24 bit
// color data for each pixel, in GRB order
struct NeoColor {
	uint8_t green;
	uint8_t red;
	uint8_t blue;
};


/**
 * NeoStrip objects manage the buffering and assigning of
 * addressable NeoPixels
 */
class NeoStrip
{
  public:

	/**
	 * Create a NeoStrip object
	 *
	 * @param pin The mbed data pin name
	 * @param N The number of pixels in the strip
	 */
	NeoStrip(PinName pin, int N);

	/**
	 * Set an overall brightness scale for the entire strip.
	 * When colors are set using setPixel(), they are scaled
	 * according to this brightness.
	 *
	 * Because NeoPixels are very bright and draw a lot of current,
	 * the brightness is set to 0.5 by default.
	 *
	 * @param bright The brightness scale between 0 and 1.0
	 */
	void setBrightness(float bright);

	/**
	 * Set a single pixel to the specified color.
	 *
	 * This method (and all other setPixel methods) uses modulo-N arithmetic
	 * when addressing pixels. If p >= N, the pixel address will wrap around.
	 *
	 * @param p The pixel number (starting at 0) to set
	 * @param color A 24-bit color packed into a single int,
	 * using standard hex color codes (e.g. 0xFF0000 is red)
	 */
	void setPixel(int p, int color);

	/**
	 * Set a single pixel to the specified color, with red, green, and blue
	 * values in separate arguments.
	 */
	void setPixel(int p, uint8_t red, uint8_t green, uint8_t blue);

	/**
	 * Set n pixels starting at pixel p.
	 *
	 * @param p The first pixel in the strip to set.
	 * @param n The number of pixels to set.
	 * @param colors An array of length n containing the 24-bit colors for each pixel
	 */
	void setPixels(int p, int n, const int* colors);

	/**
	 * Reset all pixels in the strip to be of (0x000000)
	 */
	void clear(void);

	/**
	 * Write the colors out to the strip; this method must be called
	 * to see any hardware effect.
	 *
	 * This function disables interrupts while the strip data is being sent,
	 * each pixel takes approximately 30us to send, plus a 50us reset pulse
	 * at the end.
	 */
	void write(void);

  protected:
	NeoColor* strip;	// pixel data buffer modified by setPixel() and used by neo_out()
	int N;				// the number of pixels in the strip
	int Nbytes;			// the number of bytes of pixel data (always N*3)
	float bright;		// the master strip brightness
	gpio_t gpio;		// gpio struct for initialization and getting register addresses
};

#endif
