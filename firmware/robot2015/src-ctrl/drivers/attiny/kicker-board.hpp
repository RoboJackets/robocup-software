#pragma once

#include "mbed.h"
#include "AVR910.h"
#include <string>

#define ATTINY13A_PAGE_SIZE 16
#define ATTINY13A_NUM_PAGES 32

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
     *loaded by the flash() method
     */
    KickerBoard(PinName mosi, PinName miso, PinName sck, PinName nReset,
                const std::string& progFilename);

    /**
     * @brief Reflashes the program on the kicker board MCU with the file
     *specified in the constructor.
     *
     * @param onlyIfDifferent If true, compares the program file to the MCU's
     *flash and only goes
     * through the flashing process if they're different.
     * @param verbose If verbose, debug log messages are printed to stdout
     * @return Zero indicates success, otherwise there was an error.
     */
    int flash(bool onlyIfDifferent = true, bool verbose = true);

protected:
    /**
     * @brief Uses the given function to check if it's return value equals the
     *expected value.
     *
     * @return True if the return value was the expected value, false otherwise
     */
    bool verify_param(const char* name, int expected,
                      int (AVR910::*paramMethod)(), bool verbose = false);

private:
    std::string _filename;
    DigitalOut _nReset;
};
