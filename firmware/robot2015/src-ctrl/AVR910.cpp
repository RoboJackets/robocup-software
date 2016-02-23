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

//Serial debug(USBTX, USBRX);

AVR910::AVR910(PinName mosi,
        PinName miso,
        PinName sclk,
        PinName nReset) : spi_(mosi, miso, sclk), nReset_(nReset) {

    //Slow frequency as default to ensure no errors from
    //trying to run it too fast. Increase as appropriate.
    spi_.frequency(32000);
    spi_.format(8, 0);

    int response = 0;

    //Enter serial programming mode by pulling reset line low.
    nReset_ = 0;

    //Wait 20ms before issuing first command.
    wait_ms(20);

    //Issue a programming enable command.
    response = enableProgramming();

    //Simple debugging to see if we get trapped
    //in an infinite loop.
    DigitalOut working(LED1);
    working = 1;

    //TODO: introduce a timeout.
    while (response < 0) {

        //Give nReset a positive pulse.
        nReset_ = 1;
        wait_ms(20);
        nReset_ = 0;
        wait_ms(20);

        //Issue another programming enable.
        response = enableProgramming();

    }

    working = 0;

}

int AVR910::program(FILE* binary, int pageSize, int numPages) {

    //Clear memory contents.
    chipErase();

    char pageOffset = 0;
    int  pageNumber = 0;
    int  address    = 0;
    int  c          = 0;
    int  highLow    = 0;

    /* bool high = 1; */
    /* int numCols = 0; */
    /* while ((c = getc(binary)) != EOF) { */
    /*     printf("%02x", c); */
    /*     if (!high) { */
    /*         if (numCols++ == 7) { */
    /*             printf("\r\n"); */
    /*             numCols = 0; */
    /*         } else { */
    /*             printf(" "); */
    /*         } */
    /*     } */
    /*     high = !high; */
    /*     for (int i = 0; i < 0; i++) {}; */
    /*     //wait(0.2); */
    /* } */
    /* return 0; */

    //We're dealing with paged memory.
    if (numPages > 1) {

        while ((c = getc(binary)) != EOF) {

            //Page is fully loaded, time to write it to flash.
            if (pageOffset == (pageSize)) {
                writeFlashMemoryPage(pageNumber);

                pageNumber++;
                if (pageNumber > numPages) {
                    return -1;
                    // break;
                }
                pageOffset = 0;
            }

            //Write low byte.
            if (highLow == 0) {
                loadMemoryPage(WRITE_LOW_BYTE, pageOffset, c);
                highLow = 1;
            }
            //Write high byte.
            else {
                loadMemoryPage(WRITE_HIGH_BYTE, pageOffset, c);
                highLow = 0;
                pageOffset++;
            }

        }

    }
    //We're dealing with non-paged memory.
    else {

        while ((c = getc(binary)) != EOF) {

            //Write low byte.
            if (highLow == 0) {
                writeFlashMemoryByte(WRITE_LOW_FLASH_BYTE, address, c);
                highLow = 1;
            }
            //Write high byte.
            else {
                writeFlashMemoryByte(WRITE_HIGH_FLASH_BYTE, address, c);
                highLow = 0;
                address++;

                //Page size is our memory size in the non-paged memory case.
                //Therefore if we've gone beyond our size break because we
                //don't have any more room.
                if (address > pageSize) {
                    break;
                }

            }

        }

    }

    //We might have partially filled up a page.
    writeFlashMemoryPage(pageNumber);

    int success = -1;
    //int success = 0;
    success = checkMemory(pageNumber, binary);
    
    nReset_ = 0;
    wait_ms(20);
    //Leave serial programming mode by pulling reset line high.
    nReset_ = 1;

    return success;

}

void AVR910::setFrequency(int frequency) {

    spi_.frequency(frequency);

}

int AVR910::enableProgramming(void) {

    int response = 0;
    int error    = 0;

    //Programming Enable Command: 0xAC, 0x53, 0x00, 0x00
    //Byte two echo'd back in byte three.
    spi_.write(0xAC);

    spi_.write(0x53);

    response = spi_.write(0x00);

    if (response == 0x53) {
        error =  0;
    } else {
        error = -1;
    }

    spi_.write(0x00);

    return error;

}

void AVR910::poll(void) {

    int response = 0;

    do {
        spi_.write(0xF0);
        spi_.write(0x00);
        spi_.write(0x00);
        response = spi_.write(0x00);
    } while ((response & 0x01) != 0);

}

int AVR910::readVendorCode(void) {

    int response = 0;

    //Issue read signature byte command.
    //Address 0x00 is vendor code.
    spi_.write(0x30);
    spi_.write(0x00);
    spi_.write(0x00);
    response = spi_.write(0x00);

    return response;

}

int AVR910::readPartFamilyAndFlashSize(void) {

    int response = 0;

    //Issue read signature byte command.
    //Address 0x01 is part family and flash size code.
    spi_.write(0x30);
    spi_.write(0x00);
    spi_.write(0x01);
    response = spi_.write(0x00);

    return response;

}

int AVR910::readPartNumber(void) {

    int response = 0;

    //Issue read signature byte command.
    //Address 0x02 is part number code.
    spi_.write(0x30);
    spi_.write(0x00);
    spi_.write(0x02);
    response = spi_.write(0x00);

    return response;

}

void AVR910::chipErase(void) {

    //Issue chip erase command.
    spi_.write(0xAC);
    spi_.write(0x80);
    spi_.write(0x00);
    spi_.write(0x00);

    poll();

    //Temporarily release reset line.
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
    spi_.write((pageNumber >> 2) & 0x3F);
    spi_.write((pageNumber & 0x03) << 6);
    spi_.write(0x00);

    poll();

}

char AVR910::readProgramMemory(int highLow, char pageNumber, char pageOffset) {

    int response = 0;

    spi_.write(highLow);
    spi_.write((pageNumber >> 2) & 0x3F);
    spi_.write(((pageNumber & 0x03) << 6) | (pageOffset & 0x3F));
    response = spi_.write(0x00);

    poll();

    return response;

}

int AVR910::checkMemory(int numPages, FILE* binary) {

    int success  = 0;
    int response = 0;
    char c       = 0;

    // Go back to the beginning of the binary file.
    fseek(binary, 0, SEEK_SET);
    int colnum = 0;
    for (int i = 0; i < numPages; i++) {
        for (int j = 0; j < PAGE_SIZE; j++) {
            c = getc(binary);
            //Read program memory low byte.
            response = readProgramMemory(READ_LOW_BYTE, i, j);
            //printf("Low byte: 0x%02x\r\n", response);
            //printf("%d %d\r\n", response, c);
            //printf("%02x\r\n", response);
            if ( c != response ) {
                //printf("Page %i low byte %i: 0x%02x\r\n", i, j, response);
                //printf("Correct byte is 0x%02x\r\n", c);
                success = -1;
            }
            c = getc(binary);
            //Read program memory high byte.
            response = readProgramMemory(READ_HIGH_BYTE, i, j);
            //printf("%02x\r\n ", response);
            
            //printf("High byte: 0x%02x\r\n", response);
            //printf("%d %d\r\n", response, c);

            if ( c != response ) {
                //printf("Page %i high byte %i: 0x%02x\r\n", i, j, response);
                //printf("Correct byte is 0x%02x\r\n", c);
                success = -1;
            }
            //for (int i = 0; i < 0; i++) {}
        }
    }
    return success;

}
