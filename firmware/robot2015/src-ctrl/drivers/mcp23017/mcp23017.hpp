#pragma once

#include <mbed.h>
#include <memory>

#include "pins-ctrl-2015.hpp"
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

    MCP23017(PinName sda, PinName scl, int i2cAddress);

    static std::shared_ptr<MCP23017>& Instance();

    /** Reset MCP23017 device to its power-on state
     */
    void reset();

    /** Write a 0/1 value to an output bit
     *
     * @param   value         0 or 1
     * @param   bit_number    bit number range 0 --> 15
     */
    void writeBit(int value, int bit_number);

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
    uint8_t readBit(int bit_number);

    /** Configure an MCP23017 device
     *
     * @param   dir_config         data direction value (1 = input, 0 = output)
     * @param   pullup_config      100k pullup value (1 = enabled, 0 = disabled)
     * @param   polarity_config    polarity value (1 = flip, 0 = normal)
     */
    void config(uint16_t dir_config, uint16_t pullup_config,
                uint16_t polarity_config);

    void writeRegister(MCP23017::Register regAddress, uint8_t val);
    uint16_t readRegister(MCP23017::Register regAddress);

    typedef enum { DIR_OUTPUT = 0, DIR_INPUT = 1 } PinMode;
    void pinMode(int pin, PinMode mode);
    void digitalWrite(int pin, int val);
    int digitalRead(int pin);

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

    int read();

    void write(int data);

private:
    static std::shared_ptr<MCP23017> instance;

    I2CMasterRtos _i2c;
    int _i2cAddress;  // physical I2C address

    // Cached copies of the register values
    uint16_t _cachedGPIO, _cachedIODIR, _cachedGPPU, _cachedIPOL;
};
