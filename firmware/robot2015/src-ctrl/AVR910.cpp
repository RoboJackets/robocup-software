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

#include "AVR910.hpp"


AVR910::AVR910(PinName mosi,
        PinName miso,
        PinName sclk,
        PinName nReset) : spi_(mosi, miso, sclk), nReset_(nReset) {

    // Slow frequency as default to ensure no errors from
    // trying to run it too fast. Increase as appropriate.
    spi_.frequency(32000);
    spi_.format(8, 0);

    // Enter serial programming mode by pulling reset line low.
    nReset_ = 0;

    // Wait 20ms before issuing first command.
    wait_ms(20);

    // Enable programming mode on the chip
    // It's possible for it to fail, so try it a few times.
    bool enabled = false;
    for (int i = 0; i < 10; i++) {
        enabled = enableProgramming();
        if (enabled) break;

        // Give nReset a positive pulse.
        nReset_ = 1;
        wait_ms(20);
        nReset_ = 0;
        wait_ms(20);
    }

    if (!enabled) {
        printf("ERROR: AVR910 unable to enable programming mode for chip.  Further commands will fail");
    }
}

bool AVR910::program(FILE* binary, int pageSize, int numPages) {
    // Clear memory contents.
    chipErase();

    char pageOffset = 0;
    int  pageNumber = 0;
    int  address    = 0;
    int  c          = 0;
    int  highLow    = 0;

    // We're dealing with paged memory.
    if (numPages > 1) {
        while ((c = getc(binary)) != EOF) {
            // Page is fully loaded, time to write it to flash.
            if (pageOffset == (pageSize)) {
                writeFlashMemoryPage(pageNumber);

                pageNumber++;
                if (pageNumber > numPages) {
                    printf("ERROR: AVR910 binary exceeds chip memory capacity\r\n");
                    return -1;
                }
                pageOffset = 0;
            }

            // Write low byte.
            if (highLow == 0) {
                loadMemoryPage(WRITE_LOW_BYTE, pageOffset, c);
                highLow = 1;
            }
            // Write high byte.
            else {
                loadMemoryPage(WRITE_HIGH_BYTE, pageOffset, c);
                highLow = 0;
                pageOffset++;
            }

        }

    } else {
        // We're dealing with non-paged memory.

        while ((c = getc(binary)) != EOF) {

            // Write low byte.
            if (highLow == 0) {
                writeFlashMemoryByte(WRITE_LOW_FLASH_BYTE, address, c);
                highLow = 1;
            } else {
                // Write high byte.
                writeFlashMemoryByte(WRITE_HIGH_FLASH_BYTE, address, c);
                highLow = 0;
                address++;

                // Page size is our memory size in the non-paged memory case.
                // Therefore if we've gone beyond our size break because we
                // don't have any more room.
                if (address > pageSize) {
                    printf("ERROR: AVR910 binary exceeds chip memory capacity\r\n");
                    return -1;
                }
            }
        }
    }

    // We might have partially filled up a page.
    writeFlashMemoryPage(pageNumber);

    bool success = checkMemory(pageNumber, pageSize, binary);

    // Leave serial programming mode by toggling reset
    nReset_ = 0;
    wait_ms(20);
    nReset_ = 1;

    return success;
}

void AVR910::setFrequency(int frequency) {
    spi_.frequency(frequency);
}

bool AVR910::enableProgramming() {
    // Programming Enable Command: 0xAC, 0x53, 0x00, 0x00
    // Byte two echo'd back in byte three.
    spi_.write(0xAC);
    spi_.write(0x53);
    int response = spi_.write(0x00);
    spi_.write(0x00);

    if (response == 0x53) {
        return true;
    } else {
        return false;
    }
}

void AVR910::poll() {
    // Query the chip until it indicates it's ready by setting the busy bit to 0
    int response = 0;
    do {
        spi_.write(0xF0);
        spi_.write(0x00);
        spi_.write(0x00);
        response = spi_.write(0x00);
    } while ((response & 0x01) != 0);
}

int AVR910::readVendorCode() {
    // Issue read signature byte command.
    // Address 0x00 is vendor code.
    spi_.write(0x30);
    spi_.write(0x00);
    spi_.write(0x00);
    return spi_.write(0x00);
}

int AVR910::readPartFamilyAndFlashSize() {
    // Issue read signature byte command.
    // Address 0x01 is part family and flash size code.
    spi_.write(0x30);
    spi_.write(0x00);
    spi_.write(0x01);
    return spi_.write(0x00);
}

int AVR910::readPartNumber() {
    // Issue read signature byte command.
    // Address 0x02 is part number code.
    spi_.write(0x30);
    spi_.write(0x00);
    spi_.write(0x02);
    return spi_.write(0x00);
}

void AVR910::chipErase() {
    // Issue chip erase command.
    spi_.write(0xAC);
    spi_.write(0x80);
    spi_.write(0x00);
    spi_.write(0x00);

    poll();

    // Temporarily release reset line.
    nReset_ = 1;
    nReset_ = 0;
}

void AVR910::loadMemoryPage(int highLow, char address, char data) {
    spi_.write(highLow);
    spi_.write(0x00);
    spi_.write(address & 0x3F);
    spi_.write(data);

    poll();
}

void AVR910::writeFlashMemoryByte(int highLow, int address, char data) {
    spi_.write(highLow);
    spi_.write(address & 0xFF00 >> 8);
    spi_.write(address & 0x00FF);
    spi_.write(data);
}

void AVR910::writeFlashMemoryPage(char pageNumber) {
    spi_.write(0x4C);
    spi_.write(pageNumber >> 3); // top 5 bits stored in bottom of byte
    spi_.write(pageNumber << 5); // bottom 3 bits stored in top of byte
    spi_.write(0x00);

    poll();
}

char AVR910::readProgramMemory(int highLow, char pageNumber, char pageOffset) {
    spi_.write(highLow);
    spi_.write(pageNumber >> 3);
    spi_.write((pageNumber << 5) | (pageOffset & 0x3F));
    char response = spi_.write(0x00);

    poll();

    return response;
}

bool AVR910::checkMemory(int numPages, int pageSize, FILE* binary) {
    bool success  = true;

    // Go back to the beginning of the binary file.
    fseek(binary, 0, SEEK_SET);

    for (int page = 0; page < numPages; page++) {
        for (int offset = 0; offset < pageSize; offset++) {
            int response;
            char c = getc(binary);
            // Read program memory low byte.
            response = readProgramMemory(READ_LOW_BYTE, page, offset);
            if (c != response) {
                printf("Page %i low byte %i: 0x%02x\r\n", page, offset, response);
                printf("Correct byte is 0x%02x\r\n", c);
                success = false;
            }
            c = getc(binary);
            // Read program memory high byte.
            response = readProgramMemory(READ_HIGH_BYTE, page, offset);
            if (c != response) {
                printf("Page %i high byte %i: 0x%02x\r\n", page, offset, response);
                printf("Correct byte is 0x%02x\r\n", c);
                success = false;
            }
        }
    }

    return success;
}
