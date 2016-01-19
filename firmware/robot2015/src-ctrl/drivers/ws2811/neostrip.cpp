/*
 * NeoStrip.cpp
 *
 * Allen Wild
 * March 2014
 *
 * Jonathan Jones
 * November 2015
 *
 * Controls a strip of Adafruit NeoPixels, addressable RGB LEDs
 * Currently, because of the global nature of the IO register and bitmask
 * variables,
 * it is only possible to use one NeoStrip instance at a time.
 *
 * This library supports only the NXP LPC1768!
 */

#include "neostrip.hpp"

#include <algorithm>

/*
 * This function is defined in the assembly code and is declared
 * as a global function.
 */
extern "C" void neo_out(NeoColor* strip, uint32_t nBytes);

/*
 * This is where the memory for these variables is actually allocated and
 * initialized. These are declared as global symbols in the assembly.
 */
volatile uint32_t neo_bitmask = 0;
volatile uint32_t* neo_fio_reg = 0;

unsigned int NeoStrip::_objs;
float* NeoStrip::_default_bright;
NeoColor* NeoStrip::_default_color;

NeoStrip::NeoStrip(PinName pin, unsigned int N) : _n(N) {
    _strip = new NeoColor[_n];
    _neopin = new gpio_t;
    _objs++;

    if (_objs == 1) {
        _default_color = _strip;
        _default_bright = &_bright;
    } else {
        _n = 1;
    }

    // setup the pin to be an output
    gpio_init_out_ex(_neopin, pin, 0);

    // set registers and bitmask for pin registers
    neo_fio_reg = static_cast<volatile uint32_t*>(_neopin->reg_dir);
    neo_bitmask = static_cast<volatile uint32_t>(_neopin->mask);
}

NeoStrip::~NeoStrip() {
    delete _neopin;
    delete[] _strip;
    _objs--;
}

void NeoStrip::brightness(float bright) { _bright = bright; }

float NeoStrip::brightness() { return _bright; }

void NeoStrip::setPixel(size_t p, int color) {
    int red = (color & 0xFF0000) >> 16;
    int green = (color & 0x00FF00) >> 8;
    int blue = (color & 0x0000FF);
    setPixel(p, red, green, blue);
}

void NeoStrip::setPixel(size_t p, unsigned int red, unsigned int green, unsigned int blue) {
    // set the given pixel's RGB values
    // the array is indexed modulo _N to avoid overflow
    _strip[p % _n].red = static_cast<uint8_t>(static_cast<float>(red) * _bright);
    _strip[p % _n].green = static_cast<uint8_t>(static_cast<float>(green) * _bright);
    _strip[p % _n].blue = static_cast<uint8_t>(static_cast<float>(blue) * _bright);
}

void NeoStrip::setPixels(size_t p, size_t n, const int* colors) {
    int r, g, b;
    size_t nMin = static_cast<size_t>(std::min(n, _n));
    for (size_t i = 0; i < nMin; i++) {
        r = (colors[i] & 0xFF0000) >> 16;
        g = (colors[i] & 0x00FF00) >> 8;
        b = colors[i] & 0x0000FF;
        setPixel(p + i, r, g, b);
    }
}

void NeoStrip::clear() {
    for (size_t i = 0; i < _n; i++)
        setPixel(i, 0);

    write();
}

void NeoStrip::write() {
    __disable_irq();          // disable interrupts
    neo_out(_strip, _n * 3);  // output to the strip
    __enable_irq();           // enable interrupts
    wait_us(50);              // wait 50us for the reset pulse
}

void NeoStrip::setFromDefaultColor() {
    _strip->red = static_cast<uint8_t>(_default_color->red * _bright);
    _strip->green = static_cast<uint8_t>(_default_color->green * _bright);
    _strip->blue = static_cast<uint8_t>(_default_color->blue * _bright);
}

void NeoStrip::setFromDefaultBrightness() { _bright = *_default_bright; }

void NeoStrip::defaultBrightness(float bright) { *_default_bright = bright; }
