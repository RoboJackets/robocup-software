#ifndef NEOPIXEL_H
#define NEOPIXEL_H

#include <stdint.h>
#include "mbed.h"
#include "BurstSPI.h"

namespace neopixel
{

/** Represent the value of a single pixel.
 *
 * Each channel uses the full 8 bits: 0x00 is fully off and 0xff is fully on.
 */
struct Pixel {
    uint8_t red;
    uint8_t green;
    uint8_t blue;
};

/** Control the byte order used by the connected pixels.
 *
 * The vast majority of NeoPixels use a GRB byte order, so this is the default.
 * A few use a RGB byte order.
 *
 * In principle, the WS281x controllers could be connected with _any_ byte
 * ordering, but only GRB and RGB are supported at the moment.
 */
enum ByteOrder {
    BYTE_ORDER_GRB,
    BYTE_ORDER_RGB,
};

/** Set the protocol mode.
 *
 * The protocol is named after the clock, as though WS8211 supports only the
 * 400kHz clock, WS8212 supports both.
 */
enum Protocol {
    PROTOCOL_800KHZ,
    PROTOCOL_400KHZ,
};

typedef void (*PixelGenerator)(Pixel* out, uint32_t index, uintptr_t extra);

/** Control an array or chain of NeoPixel-compatible RGB LEDs.
 *
 * "NeoPixel" is Adafruit's name for WS2812- and WS2811-based addressable RGB
 * LEDs. This library should work with any WS2811- or WS2812-based devices, as
 * long as they support the fast-mode (800kHz) interface.
 *
 * Most example code uses bit-banging to generate the timed signal precisely.
 * This library uses an SPI peripheral instead. The main advantage of this is
 * that the chip can service interrupts and the like without disrupting the
 * signal (as long as the interrupts don't take _too_ long). The main
 * disadvantage is that it requires the use of an SPI peripheral.
 *
 * @note SPI peripherals will tend to leave the output pin ('MOSI') floating
 * after a packet is sent. This will confuse the connected pixels, which expect
 * the line to be driven low when idle. One way to fix this is to add a 10k
 * resistor between 'MOSI' and ground so that it drops to '0' when not driven.
 * Another method is to enable the on-chip pull-down resistor on the output pin.
 * However, the mbed API only exposes this function through the DigitalIn and
 * DigitalInOut classes. If you want to use the on-chip pull-down, you'll have
 * to temporarily connect a DigitalIn peripheral _before_ creating instantiating
 * the PixelArray.
 *
 * @code
 * // Sample generator: Cycle through each colour combination, increasing the
 * // brightness each time. `extra` is used as an iteration counter.
 * void generate(neopixel::Pixel * out, uint32_t index, uintptr_t extra) {
 *   uint32_t brightness = (index + extra) >> 3;
 *   out->red   = ((index + extra) & 0x1) ? brightness : 0;
 *   out->green = ((index + extra) & 0x2) ? brightness : 0;
 *   out->blue  = ((index + extra) & 0x4) ? brightness : 0;
 * }
 *
 * int main() {
 *   // Create a temporary DigitalIn so we can configure the pull-down resistor.
 *   // (The mbed API doesn't provide any other way to do this.)
 *   // An alternative is to connect an external pull-down resistor.
 *   DigitalIn(p5, PullDown);
 *
 *   // The pixel array control class.
 *   neopixel::PixelArray array(p5);
 *
 *   uint32_t offset = 0;
 *   while (1) {
 *     array.update(generate, 100, offset++);
 *     wait_ms(250);
 *   }
 * }
 * @endcode
 */
class PixelArray
{
public:
    /** Initialize a PixelArray.
     *
     * @param out Output (SPI MOSI) pin.
     * @param byte_order The order in which to transmit colour channels.
     */
    PixelArray(PinName out,
               ByteOrder byte_order = BYTE_ORDER_GRB,
               Protocol protocol = PROTOCOL_800KHZ);

    /** Update the pixel display from a buffer.
     *
     * This update method is good in the following situations:
     * - You want to make incremental changes to a fixed frame pattern.
     * - The frame is hard (or impossible) to generate procedurally.
     * - The frame requires a lot of time to generate.
     *
     * @param buffer Pixel data to be written.
     * @param length The number of pixels to write.
     *
     * buffer[0] is written to the pixel nearest the mbed.
     * buffer[length-1] is written to the pixel furthest from the mbed.
     */
    void update(Pixel buffer[], uint32_t length);

    /** Update a pixel chain using the callback to generate the value for each
     * pixel.
     *
     * This update method is good in the following situations:
     * - You have a lot of pixels to drive and don't have enough RAM to buffer
     *   them all.
     * - You want to display a frame pattern that can be generated procedurally
     *   generated without intensive processing.
     *
     * @param generator A callback which is called to generate a value for each
     * pixel on demand. This function must be fairly fast: if it takes more
     * than about 8-9us, the interface will reset and the display will be
     * corrupted. The exact time limits will vary between WS281x variants. As a
     * rough guide, an LPC1768 at 96MHz can (conservatively) execute about 750
     * instructions in that time.
     *
     * @param length The number of pixels to write.
     *
     * @param extra An arbitrary value to pass into the generator function. For
     * example, this is a good way to pass an animation time index to the
     * generator function.
     */
    void update(PixelGenerator generator, uint32_t length, uintptr_t extra);

private:
    BurstSPI spi_;
    ByteOrder byte_order_;
    Protocol protocol_;
    
    static int const latch_time_us_ = 50;
    
    void send_pixel(Pixel& pixel);
};

}

#endif
