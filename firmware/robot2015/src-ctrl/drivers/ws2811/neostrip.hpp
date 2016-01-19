#pragma once
/**
 * NeoStrip.h
 *
 * Allen Wild
 * March 2014
 *
 * Jonathan Jones
 * November 2015
 *
 * Library for the control of Adafruit NeoPixel addressable RGB LEDs.
 */
#include <cstddef>

#include <mbed.h>

#ifndef TARGET_LPC1768
#error NeoStrip only supports the NXP LPC1768!
#endif

/**
 * These store register info that the
 * assembly code needs.
 */
extern volatile uint32_t* neo_fio_reg;
extern volatile uint32_t neo_bitmask;

/**
 * NeoColor struct definition to hold 24 bit
 * color data for each pixel, in GRB order.
 */
struct NeoColor {
    uint8_t green;
    uint8_t red;
    uint8_t blue;
} __attribute__((packed));

/**
 * NeoStrip objects manage the buffering and assigning of
 * addressable NeoPixels
 */
class NeoStrip {
public:
    /**
     * Create a NeoStrip object
     *
     * @param pin The mbed data pin name
     * @param N The number of pixels in the strip
     */
    NeoStrip(PinName pin, unsigned int N = 1);

    /**
     * Deconstructor for cleaning up memory
     */
    ~NeoStrip();

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
    void brightness(float bright);
    float brightness();

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
    void setPixel(size_t p, int color);

    /**
     * Set a single pixel to the specified color, with red, green, and blue
     * values in separate arguments.
     */
    void setPixel(size_t p, unsigned int red, unsigned int green, unsigned int blue);

    /**
     * Set n pixels starting at pixel p.
     *
     * @param p The first pixel in the strip to set.
     * @param n The number of pixels to set.
     * @param colors An array of length n containing the 24-bit colors for each
     * pixel
     */
    void setPixels(size_t p, size_t n, const int* colors);

    /**
     * Reset all pixels in the strip to be of (0x000000)
     */
    void clear();

    /**
     * Write the colors out to the strip; this method must be called
     * to see any hardware effect.
     *
     * This function disables interrupts while the strip data is being sent,
     * each pixel takes approximately 30us to send, plus a 50us reset pulse
     * at the end.
     */
    void write();

    void setFromDefaultBrightness();
    void setFromDefaultColor();
    static void defaultBrightness(float bright);

protected:
    // pixel data used in setPixel() and neo_out()
    NeoColor* _strip;
    // gpio struct for pin setup, should really be static
    gpio_t* _neopin;
    // the number of pixels in the strip
    unsigned int _n;
    // brightness
    float _bright;
    // default statics since assuming
    // all objects use same hardware
    static NeoColor* _default_color;
    static float* _default_bright;
    static unsigned int _objs;
};
