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

#pragma once

/**
 * Includes
 */
#include "mbed.h"
#include <vector>

/**
 * Defines
 */

// Commands
#define ATMEL_VENDOR_CODE 0x1E
#define DEVICE_LOCKED 0x00
#define AVR_FAMILY_MASK 0x90
#define CHIP_TYPE_ATTINY13A 0x07
#define PROGRAMMING_MODE_ENABLED 0x53

#define LOAD_BYTE_BASE 0x40
#define READ_BYTE_BASE 0x20
#define WRITE_HIGH_FLASH_BYTE 0x68
#define WRITE_LOW_FLASH_BYTE 0x60

#define AVR_DEBUG_MODE 0

/**
 * AVR910 ISP
 */
class AVR910 {
public:
    /**
     * Constructor.
     *
     * @param mosi mbed pin for MOSI SPI line.
     * @param miso mbed pin for MISO SPI line.
     * @param sclk mbed pin for SCLK SPI line.
     * @param nReset mbed pin for not reset line on the ISP interface.
     *
     * Sends an enable programming command, allowing device registers to be
     * read and commands sent.
     */
    AVR910(PinName mosi, PinName miso, PinName sclk, PinName nReset);

    /**
     * Program the AVR microcontroller connected to the mbed.
     *
     * Sends a chip erase command followed by writing the binary to the AVR
     * page buffer and writing the page buffer to flash memory whenever it is
     * full.
     *
     * @param binary File pointer to the binary file to be loaded onto the
     *               AVR microcontroller.
     * @param pageSize The size of one page on the device in words. If the
     *                 device does not use paged memory, set this as the size
     *                 of memory of the device in words.
     * @param numPages The number of pages on the device. If the device does
     *                 not use paged memory, set this to 1 (default).
     *
     * @return  0 => AVR microcontroller programmed successfully.
     *         -1 => Problem during programming.
     */
    int program(FILE* binary, int pageSize, int numPages = 1);

    /**
     * Read the vendor code of the device.
     *
     * @return The vendor code - should be 0x1E for Atmel.
     *         0x00 -> Device is locked.
     */
    int vendor(void);

    /**
     * Read the part family and flash size of the device.
     *
     * @return Code indicating the family of AVR microcontrollers the device
     *comes
     *         from and how much flash memory it contains.
     *         0xFF -> Device code erased or target missing.
     *         0x01 -> Device is locked.
     */
    int family(void);

    /**
     * Read the part number.
     *
     * @return Code identifying the part number.
     *         0xFF -> Device code erased or target missing.
     *         0x02 -> Device is locked.
     */
    int partnum(void);

    /**
     * Issue an enable programming command to the AVR microcontroller.
     *
     * @param  0 to indicate programming was enabled successfully.
     *        -1 to indicate programming was not enabled.
     */
    int enable(void);
    int lock_bits(void);

protected:
    /**
     * Check the binary has been written correctly.
     *
     * @param numPages The number of pages written to the AVR microcontroller.
     * @param binary File pointer to the binary used.
     *
     * @return  0 -> No inconsistencies between binary file and AVR flash
     *memory.
     *         -1 -> Binary file was not written correctly.
     */
    int verify(FILE* binary, unsigned int pgSz, unsigned int numPgs);

private:
    /**
     * Poll the device until it has finished its current operation.
     */
    int poll(void);

    /**
     * Issue a chip erase command to the AVR microcontroller.
     */
    int erase(void);

    /**
     * Load a byte into the memory page buffer.
     *
     * @param highLow Indicate whether the byte being loaded is a high or low
     *                byte.
     * @param data The data byte to load.
     */
    int load_page(uint8_t, uint8_t, bool);

    /**
     * Write a byte into the flash memory.
     *
     * @param highLow Indicate whether the byte being loaded is a high or low
     *                byte.
     * @param address The address to load the byte at.
     * @param data The data byte to load.
     */
    int write_byte(int highLow, int address, uint8_t data);

    /**
     * Write the memory page buffer to flash memory.
     *
     * @param pageNumber The page number to write to in flash memory.
     */
    int write_page(uint8_t);

    /**
     * Read a byte from program memory.
     *
     * @param highLow Indicate whether the byte being read is a low or high
     *byte.
     * @param pageNumber The page number to read from.
     * @param pageOffset Address of byte in the page.
     *
     * @return The byte at the specified memory location.
     */
    int8_t read_byte(uint8_t pgNbr, uint8_t pgOffset, bool hl);

    uint8_t read_calibration(void);
    bool fuse(void);

    SPI _spi;
    DigitalOut _nReset;

    bool _isReady;
};