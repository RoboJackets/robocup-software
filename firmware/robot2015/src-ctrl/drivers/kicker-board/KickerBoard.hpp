#pragma once

#include <mbed.h>
#include <string>
#include "AVR910.hpp"

// ATtiny84a
#define AVR_FAMILY_MASK 0xF0
#define AVR_FAMILY_ID 0x90
#define ATTINY84A_DEVICE_ID 0x0C
#define ATTINY84A_PAGESIZE 32  // Size in words (word = 2 bytes)
#define ATTINY84A_NUM_PAGES 128

/**
 * @brief A class for interfacing with the kicker board, which is based on an
 * AVR chip.
 */
class KickerBoard : public AVR910 {
public:
    /**
     * @brief Constructor for KickerBoard
     *
     * @param mosi mbed pin for MOSI SPI line.
     * @param miso mbed pin for MISO SPI line.
     * @param sclk mbed pin for SCLK SPI line.
     * @param nReset mbed pin for not reset line on the ISP interface.
     * @param progFilename Path to kicker program binary file that will be
     *     loaded by the flash() method
     */
    KickerBoard(PinName mosi, PinName miso, PinName sck, PinName nReset,
                const std::string& progFilename);

    /**
     * @brief Reflashes the program on the kicker board MCU with the file
     *     specified in the constructor.
     *
     * @param onlyIfDifferent If true, compares the program file to the MCU's
     *     flash and only goes through the flashing process if they're
     *     different.
     * @param verbose If verbose, debug log messages are printed to stdout
     * @return True if flashing was successful
     */
    bool flash(bool onlyIfDifferent = true, bool verbose = false);

protected:
    /**
     * @brief Uses the given function to check if it's return value equals the
     * expected value.
     *
     * @return True if the return value was the expected value, false otherwise
     */
    bool verify_param(const char* name, char expected,
                      int (AVR910::*paramMethod)(), char mask = 0xFF,
                      bool verbose = false);

private:
    std::string _filename;
};
