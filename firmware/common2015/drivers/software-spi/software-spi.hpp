#pragma once

/** A software implemented SPI that can use any digital pins
 *
 * Useful when don't want to share a single SPI hardware among attached devices
 * or when pinout doesn't match exactly to the target's SPI pins
 *
 * @code
 * #include "mbed.h"
 * #include "software-spi.hpp"
 *
 * SoftwareSPI spi(p5, p6, p7); // mosi, miso, sck
 *
 * int main()
 * {
 *     DigitalOut cs(p8);
 *     spi.format(8, 0);
 *
 *     cs.write(0);
 *     spi.write(0x9f);
 *     int jedecid = (spi.write(0) << 16) | (spi.write(0) << 8) | spi.write(0);
 *     cs.write(1);
 * }
 * @endcode
 */
class SoftwareSPI {
private:
    DigitalOut* mosi;
    DigitalIn* miso;
    DigitalOut* sck;
    int port;
    int bits;
    int mode;
    int polarity;  // idle clock value
    int phase;  // 0=sample on leading (first) clock edge, 1=trailing (second)
    int freq;

    uint32_t mosi_timer_base, mosi_pin_mode, sck_timer_base, sck_pin_mode;

public:
    /** Create SoftwareSPI object
     *
     *  @param mosi_pin
     *  @param miso_pin
     *  @param sck_pin
     */
    SoftwareSPI(PinName mosi_pin, PinName miso_pin, PinName sck_pin,
                int bit_width = 8);

    /** Destructor */
    ~SoftwareSPI();

    /** Specify SPI format
     *
     *  @param bits  8 or 16 are typical values
     *  @param mode  0, 1, 2, or 3 phase (bit1) and idle clock (bit0)
     */
    void format(int bits, int mode = 0);

    /** Write data and read result
     *
     *  @param value  data to write (see format for bit size)
     *  returns value read from device
     */
    int write(int value);
};
