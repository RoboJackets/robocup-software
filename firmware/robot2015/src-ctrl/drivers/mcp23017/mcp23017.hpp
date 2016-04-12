#pragma once

#include <mbed.h>
#include <memory>

#include "I2CMasterRtos.hpp"

/**
 * Allow access to an I2C-connected MCP23017 16-bit I/O extender chip
 */
class MCP23017 {
public:
    // Register defines from data sheet - we set IOCON.BANK to 0
    // as it is easier to manage the registers sequentially.
    typedef enum {
        IODIR = 0x00,
        IPOL = 0x02,
        GPINTEN = 0x04,
        DEFVAL = 0x06,
        INTCON = 0x08,
        IOCON = 0x0A,
        GPPU = 0x0C,
        INTF = 0x0E,
        INTCAP = 0x10,
        GPIO = 0x12,
        OLAT = 0x14
    } Register;

    typedef enum {
        // Port A bit masks
        PinA0 = 0,
        PinA1 = 1,
        PinA2 = 2,
        PinA3 = 3,
        PinA4 = 4,
        PinA5 = 5,
        PinA6 = 6,
        PinA7 = 7,
        // Port B bit masks
        PinB0 = 8,
        PinB1 = 9,
        PinB2 = 10,
        PinB3 = 11,
        PinB4 = 12,
        PinB5 = 13,
        PinB6 = 14,
        PinB7 = 15
    } ExpPinName;

    MCP23017(PinName sda, PinName scl, int i2cAddress);

    /** Reset MCP23017 device to its power-on state
     */
    void reset();

    /** Write a 0/1 value to an output bit
     *
     * @param   value         0 or 1
     * @param   bit_number    bit number range 0 --> 15
     */
    void writePin(int value, MCP23017::ExpPinName pin);

    /** Write a masked 16-bit value to the device
     *
     * @param   data    16-bit data value
     * @param   mask    16-bit mask value
     */
    void writeMask(uint16_t data, uint16_t mask);

    int readMask(uint16_t mask);

    /** Read a 0/1 value from an input bit
     *
     * @param   bit_number    bit number range 0 --> 15
     * @return                0/1 value read
     */
    uint8_t readPin(MCP23017::ExpPinName pin);

    /** Configure an MCP23017 device
     *
     * @param   dir_config         data direction value (1 = input, 0 = output)
     * @param   pullup_config      100k pullup value (1 = enabled, 0 = disabled)
     * @param   polarity_config    polarity value (1 = flip, 0 = normal)
     */
    void config(uint16_t dir_config, uint16_t pullup_config,
                uint16_t polarity_config);

    void writeRegister(MCP23017::Register regAddress, uint16_t val);
    uint16_t readRegister(MCP23017::Register regAddress);

    typedef enum { DIR_OUTPUT = 0, DIR_INPUT = 1 } PinMode;
    void pinMode(ExpPinName pin, PinMode mode);
    void digitalWrite(ExpPinName pin, int val);
    int digitalRead(ExpPinName pin);

    // These provide a more advanced mapping of the chip functionality
    // See the data sheet for more information on what they do

    // Returns a word with the current pin states (ie contents of the GPIO
    // register)
    uint16_t digitalWordRead();

    // Allows you to write a word to the GPIO register
    void digitalWordWrite(uint16_t w);

    // Sets up the polarity mask that the MCP23017 supports
    // if set to 1, it will flip the actual pin value.
    void inputPolarityMask(uint16_t mask);

    // Sets which pins are inputs or outputs (1 = input, 0 = output) NB Opposite
    // to arduino's definition for these
    void inputOutputMask(uint16_t mask);

    // Allows enabling of the internal 100k pullup resisters (1 = enabled, 0 =
    // disabled)
    void internalPullupMask(uint16_t mask);

    void write(int data);

private:
    I2CMasterRtos _i2c;
    int _i2cAddress;  // physical I2C address

    // Cached copies of the register values
    uint16_t _cachedGPIO, _cachedIODIR, _cachedGPPU, _cachedIPOL;
};
