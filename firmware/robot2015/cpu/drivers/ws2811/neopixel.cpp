#if 0

#include <stdint.h>
#include "neopixel.hpp"

namespace neopixel
{

void PixelArray::SendFourBits(uint32_t bits)
{
    // Encode '0' bits as 100 and '1' bits as 110.
    // We have this bit pattern: 00000000abcd
    // We want this bit pattern: 1a01b01c01d0
    uint32_t ac = (bits * 0x088) &        // 0abcdabcd000
                  0x410; // 0a00000c0000

    uint32_t bd = (bits * 0x022) &        // 000abcdabcd0
                  0x082; // 0000b00000d0

    static uint32_t const base = 04444;   // 100100100100

    fastWrite(base | ac | bd);        // 1a01b01c01d0
}

void PixelArray::SendEightBits(uint8_t bits)
{
    int zero = 0x300;  // Encode zero as 0b1100000000
    int one = 0x3e0;   // Encode one as 0b1111100000

    for (int i = 128; i >= 1; i >>= 1)
        fastWrite((bits & i) ? one : zero);
}

PixelArray::PixelArray(PinName out, ByteOrder byte_order, Protocol protocol)
    : BurstSPI::BurstSPI(out, NC, NC),
      byte_order_(byte_order),
      protocol_(protocol)
{
    if (protocol_ == PROTOCOL_800KHZ) {
        // 800kHz bit encodings:
        //  '0': ----________
        //  '1': --------____
        // The period is 1.25us, giving a basic frequency of 800kHz.
        // Getting the mark-space ratio right is trickier, though. There are a number
        // of different timings, and the correct (documented) values depend on the
        // controller chip.
        //
        // The _real_ timing restrictions are much simpler though, and someone has
        // published a lovely analysis here:
        //   http://cpldcpu.wordpress.com/2014/01/14/light_ws2812-library-v2-0-part-i-understanding-the-ws2812/
        //
        // In summary:
        // - The period should be at least 1.25us.
        // - The '0' high time can be anywhere from 0.0625us to 0.5us.
        // - The '1' high time should be longer than 0.625us.
        //
        // These constraints are easy to meet by splitting each bit into three and packing them into SPI packets.
        //  '0': 100             mark: 0.42us, space: 0.83us
        //  '1': 110             mark: 0.83us, space: 0.42us
        // spi_.frequency(2400000);  // 800kHz * 3
        // spi_.format(12);          // Send four NeoPixel bits in each packet.
        frequency(2400000);
        format(12);
    } else {
        // 400kHz bit encodings:
        //  '0': --________
        //  '1': -----_____
        //
        // Timing requirements are derived from this document:
        //   http://www.adafruit.com/datasheets/WS2811.pdf
        //
        // The period is 2.5us, and we use a 10-bit packet for this encoding:
        //  '0': 1100000000      mark: 0.5us, space: 2us
        //  '1': 1111100000      mark: 1.25us, space: 1.25us
        // spi_.frequency(4000000);  // 400kHz * 10
        // spi_.format(10);          // Send one NeoPixel bit in each packet.
        frequency(4000000);
        format(10);
    }
}

void PixelArray::send_pixel(Pixel& pixel)
{
    // Pixels are sent as follows:
    // - The first transmitted pixel is the pixel closest to the transmitter.
    // - The most significant bit is always sent first.
    //
    // g7,g6,g5,g4,g3,g2,g1,g0,r7,r6,r5,r4,r3,r2,r1,r0,b7,b6,b5,b4,b3,b2,b1,b0
    // \_____________________________________________________________________/
    //                           |      _________________...
    //                           |     /   __________________...
    //                           |    /   /   ___________________...
    //                           |   /   /   /
    //                          GRB,GRB,GRB,GRB,...
    //
    // For BYTE_ORDER_RGB, the order of the first two bytes are reversed.

    uint8_t byte0 = (byte_order_ == BYTE_ORDER_RGB) ? pixel.red : pixel.green;
    uint8_t byte1 = (byte_order_ == BYTE_ORDER_RGB) ? pixel.green : pixel.red;

    if (protocol_ == PROTOCOL_800KHZ) {
        SendFourBits((byte0 >> 4) & 0xf);
        SendFourBits((byte0 >> 0) & 0xf);
        SendFourBits((byte1 >> 4) & 0xf);
        SendFourBits((byte1 >> 0) & 0xf);
        SendFourBits((pixel.blue >> 4) & 0xf);
        SendFourBits((pixel.blue >> 0) & 0xf);
    } else {
        SendEightBits(byte0);
        SendEightBits(byte1);
        SendEightBits(pixel.blue);
    }
}

void PixelArray::update(Pixel buffer[], uint32_t length)
{
    for (size_t i = 0; i < length; i++)
        send_pixel(buffer[i]);

    wait_us(latch_time_us_);
}

void PixelArray::update(PixelGenerator generator, uint32_t length, uintptr_t extra)
{
    for (size_t i = 0; i < length; i++) {
        Pixel out;
        generator(&out, i, extra);
        send_pixel(out);
    }

    wait_us(latch_time_us_);
}

}   // namespace

#endif