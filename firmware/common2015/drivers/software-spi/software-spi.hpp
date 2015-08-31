/* SWSPI, Software SPI library
 * Copyright (c) 2012-2014, David R. Van Wagner, http://techwithdave.blogspot.com
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#pragma once

/** A software implemented SPI that can use any digital pins
 *
 * Useful when don't want to share a single SPI hardware among attached devices
 * or when pinout doesn't match exactly to the target's SPI pins
 *
 * @code
 * #include "mbed.h"
 * #include "SoftwareSPI.hpp"
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
class SoftwareSPI
{
  private:
    DigitalOut* mosi;
    DigitalIn* miso;
    DigitalOut* sck;
    int port;
    int bits;
    int mode;
    int polarity; // idle clock value
    int phase; // 0=sample on leading (first) clock edge, 1=trailing (second)
    int freq;

  public:
    /** Create SoftwareSPI object
     *
     *  @param mosi_pin
     *  @param miso_pin
     *  @param sck_pin
     */
    SoftwareSPI(PinName mosi_pin, PinName miso_pin, PinName sck_pin, int bit_width = 8);

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
