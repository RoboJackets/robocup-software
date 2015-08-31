#include "mbed.h"

#include "software-spi.hpp"

SoftwareSPI::SoftwareSPI(PinName mosi_pin, PinName miso_pin, PinName sck_pin, int bit_width)
{
    mosi = new DigitalOut(mosi_pin);
    miso = new DigitalIn(miso_pin);
    sck = new DigitalOut(sck_pin);
    format(bit_width);
}

SoftwareSPI::~SoftwareSPI()
{
    delete mosi;
    delete miso;
    delete sck;
}

void SoftwareSPI::format(int bits, int mode)
{
    this->bits = bits;
    this->mode = mode;
    polarity = (mode >> 1) & 1;
    phase = mode & 1;
    sck->write(polarity);
}

int SoftwareSPI::write(int value)
{
    int read = 0;

    for (int bit = bits - 1; bit >= 0; --bit) {
        mosi->write(((value >> bit) & 0x01) != 0);

        if (phase == 0) {
            if (miso->read())
                read |= (1 << bit);
        }

        sck->write(!polarity);

        if (phase == 1) {
            if (miso->read())
                read |= (1 << bit);
        }

        sck->write(polarity);
    }

    return read;
}

