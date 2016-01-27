/**
 * @author Aaron Berk
 *
 * @section LICENSE
 *
 * Copyright (c) 2010 Aaron Berk
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
 *
 * @section DESCRIPTION
 *
 * Program AVR chips with the AVR910 ISP (in-system programming) protocol,
 * using an mbed.
 *
 * AVR910 Application Note:
 *
 * http://www.atmel.com/dyn/resources/prod_documents/doc0943.pdf
 */

/**
 * Includes
 */
#include "AVR910.h"

AVR910::AVR910(PinName mosi, PinName miso, PinName sclk, PinName nReset)
    : _spi(mosi, miso, sclk), _nReset(nReset) {
    // Slow frequency as default to ensure no errors from
    // trying to run it too fast. Increase as appropriate.
    _spi.frequency(80000);
    _spi.format(8, 0);

    _isReady = false;

    // Pulse the reset line before beginning
    _nReset = 1;
    wait_ms(20);
    _nReset = !_nReset;  // = 0
}

bool AVR910::fuse(void) {
    if (!_isReady) return -1;

    uint8_t buf[3] = {0x50, 0x00, 0x00};

    for (int i = 0; i < 3; i++) _spi.write(buf[i]);

    return (_spi.write(0x00) > 0 ? 0 : 0);
}

int AVR910::program(FILE* binary, int pageSize, int numPages) {
    if (!_isReady) return -1;

    // Clear memory contents and reenter programming mode
    erase();
    enable();

    uint8_t pageOffset = 0;
    int pageNbr = 0;
    uint8_t c = 0;
    bool highLow = false;  // start loading LOW_BYTE first, then HIGH_BYTE

#if AVR_DEBUG_MODE
    std::printf("  Calibration Byte: 0x%02X\r\n", read_calibration());
#endif

    uint8_t val = read_byte(0, 0, 0);
    if (val == 0xFF) {
#if AVR_DEBUG_MODE
        std::printf("  Memory successfully cleared\r\n");
#endif
    } else {
#if AVR_DEBUG_MODE
        std::printf("  Memory failed to clear. Memory value: 0x%02X\r\n", val);
#endif
        return -1;
    }

    // Go back to the beginning of the binary file.
    fseek(binary, 0, SEEK_SET);

    // We're dealing with paged memory.
    if (numPages > 1) {
#if AVR_DEBUG_MODE
        std::printf("  Writing paged memory\r\n");
#endif

        uint8_t low_byte = 0;
        while ((c = getc(binary)) != EOF) {
            // Page is fully loaded, time to write it to flash.
            if (pageSize <= pageOffset) {
                write_page(pageNbr);
                pageNbr++;

                if (numPages <= pageNbr) {
                    break;
                } else {
                    pageOffset = 0;
                }
            }

            if (!highLow) {  // if HIGH_BYTE, store it and read in the LOW_BYTE
                             // also in the next iteration
                low_byte = c;
            } else {
                // Write either high or low byte
                load_page(pageOffset, low_byte, false);
                load_page(pageOffset, c, true);
                pageOffset++;
            }

            // Signal to flip operations every other iteration
            highLow = !highLow;
        }
    }

    // We're dealing with non-paged memory.
    /*
    else {
    #if AVR_DEBUG_MODE
        std::printf("  Writing non-paged memory\r\n");
    #endif
        while ((c = getc(binary)) != EOF) {
            //Write low byte.
            if (!highLow) {
                // write_byte(WRITE_LOW_FLASH_BYTE, address, c);
            }
            //Write high byte.
            else {
                // write_byte(WRITE_HIGH_FLASH_BYTE, address, c);
                address++;
                //Page size is our memory size in the non-paged memory case.
                //Therefore if we've gone beyond our size break because we
                //don't have any more room.
                if (address > pageSize) {
                    break;
                }
            }
            highLow = !highLow;
        }
    }
    */

    // We might have partially filled up a page.
    write_page(pageNbr);

    // Read all the bytes back now
    int verification = verify(binary, pageSize, numPages);

    // Leave serial programming mode by pulling reset line high.
    _nReset = 1;

    _isReady = false;

    return verification;
}

int AVR910::enable(void) {
    if (_isReady) return 0;

    uint8_t response = 0;
    uint8_t buf[3] = {0xAC, 0x53, 0x00};

    for (int j = 0; j < 20; j++) {
        // Give nReset a positive pulse.
        _nReset = 1;
        wait_ms(20);
        _nReset = !_nReset;
        wait_ms(20);

        for (int i = 0; i < 3; i++) response = _spi.write(buf[i]);

        _spi.write(0);

        if (response & PROGRAMMING_MODE_ENABLED) {
            _isReady = true;
            break;
        }
    }

    return ((response & PROGRAMMING_MODE_ENABLED) ? 0 : -1);
}

int AVR910::poll(void) {
    if (!_isReady) return -1;

    DigitalOut temp(p20, 1);

    int response;
    uint8_t buf[4] = {0xF0, 0x00, 0x00, 0x00};

    do {
        for (int i = 0; i < 4; i++) response = _spi.write(buf[i]);
    } while (response & 0x01);

    temp = 0;

    return 0;
}

int AVR910::vendor(void) {
    if (!_isReady) return -1;

    uint8_t buf[3] = {0x30, 0x00, 0x00};

    for (int i = 0; i < 3; i++) _spi.write(buf[i]);

    return _spi.write(0);
}

int AVR910::family(void) {
    if (!_isReady) return -1;

    uint8_t buf[3] = {0x30, 0x00, 0x01};

    for (int i = 0; i < 3; i++) _spi.write(buf[i]);

    return _spi.write(0);
}

int AVR910::partnum(void) {
    if (!_isReady) return -1;

    uint8_t buf[3] = {0x30, 0x00, 0x02};

    for (int i = 0; i < 3; i++) _spi.write(buf[i]);

    return _spi.write(0);
}

int AVR910::lock_bits(void) {
    if (!_isReady) return -1;

    uint8_t buf[3] = {0x58, 0x00, 0x00};

    for (int i = 0; i < 3; i++) _spi.write(buf[i]);

    return _spi.write(0);
}

int AVR910::erase(void) {
    if (!_isReady) return -1;

    uint8_t buf[4] = {0xAC, 0x80, 0x00, 0x00};

    for (int i = 0; i < 4; i++) _spi.write(buf[i]);

    poll();

    _nReset = !_nReset;
    _nReset = !_nReset;

    // Have to toggle reset pin to exit clearing memory mode, so device is out
    // of programming mode now
    _isReady = false;

    return 0;
}

int AVR910::load_page(uint8_t addr, uint8_t byte, bool hl) {
    if (!_isReady) return -1;

    uint8_t buf[4] = {LOAD_BYTE_BASE | (hl << 3), 0x00, addr & 0x0F, byte};

    for (int i = 0; i < 4; i++) _spi.write(buf[i]);

    poll();

    return 0;
}
/*
int AVR910::write_byte(int highLow, int address, uint8_t data)
{
    if(!_isReady)
        return -1;
    _spi.write(highLow);
    _spi.write(address & 0xFF00 >> 8);
    _spi.write(address & 0x00FF);
    _spi.write(data);
    wait_ms(5);
    poll();
}
*/

int AVR910::write_page(uint8_t byteIndex) {
    if (!_isReady) return -1;

    uint8_t buf[4] = {0x4C, (byteIndex >> 4) & 0x01, (byteIndex << 4) & 0xF0,
                      0x00};

    for (int i = 0; i < 4; i++) _spi.write(buf[i]);

    wait_ms(5);
    poll();

    return 0;
}

int8_t AVR910::read_byte(uint8_t pgNbr, uint8_t pgOffset, bool hl) {
    if (!_isReady) return -1;

    uint8_t buf[4] = {READ_BYTE_BASE | (hl << 3), (pgNbr >> 4) & 0x01,
                      ((pgNbr & 0x0F) << 4) | (pgOffset & 0x0F), 0x00};
    uint8_t val;
    for (int i = 0; i < 4; i++) {
        val = _spi.write(buf[i]);
    }

    poll();

    return val;
}

uint8_t AVR910::read_calibration(void) {
    uint8_t buf[3] = {0x38, 0x00, 0x00};

    for (int i = 0; i < 3; i++) _spi.write(buf[i]);

    return _spi.write(0);
}

int AVR910::verify(FILE* binary, unsigned int pgSz, unsigned int numPgs) {
    int8_t nSuccess = 0;
    uint8_t c;
    uint16_t error_array[2][numPgs];

    // Go back to the beginning of the binary file.
    fseek(binary, 0, SEEK_SET);

    for (unsigned int i = 0; i < numPgs; i++) {
        error_array[0][i] =
            0;  // initialize each indexed value as the loop iterates through

        // Loop through all the byte indexes of the page of memory
        for (unsigned int j = 0; j < pgSz; j++) {
            for (int k = 0; k < 2; k++) {  // low byte, then high byte

                c = getc(binary);  // make sure compiler uses unsigned

                if (read_byte(i, j, k) != c) {
                    error_array[0][i]++;
                    nSuccess = -1;
                }
            }
        }
        error_array[1][i] = i;
    }

#if AVR_DEBUG_MODE
    if (nSuccess) {
        unsigned int summ = 0;

        std::printf(
            "\r\n    [MEMRY REPORT]   \r\n  ------------------\r\n  |  Page  | "
            "# Errs |\r\n  ------------------\r\n");

        for (int k = 0; k < numPgs; k++) {
            std::printf("  |  0x%02X  |   %02u   |\r\n", error_array[1][k],
                        error_array[0][k]);
            summ += error_array[0][k];
        }

        std::printf(
            "  ------------------\r\n  | Total  | %4u   |\r\n  |  Bad   |  "
            "%2.1f%% |\r\n  ------------------\r\n\r\n",
            summ,
            static_cast<float>((static_cast<float>(summ) / (i * j * 2))) * 100);
    }
#endif
    return nSuccess;
}