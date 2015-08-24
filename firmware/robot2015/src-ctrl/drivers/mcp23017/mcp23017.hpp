#pragma once

#include <mbed.h>
#include <memory>

#include "pins-ctrl-2015.hpp"


// Register defines from data sheet - we set IOCON.BANK to 0
// as it is easier to manage the registers sequentially.
enum {
    IODIR       = 0x00,
    IPOL        = 0x02,
    GPINTEN     = 0x04,
    DEFVAL      = 0x06,
    INTCON      = 0x08,
    IOCON       = 0x0A,
    GPPU        = 0x0C,
    INTF        = 0x0E,
    INTCAP      = 0x10,
    GPIO        = 0x12,
    OLAT        = 0x14
};

enum {
    DIR_OUTPUT  = 0,
    DIR_INPUT   = 1
};

/** MCP23017 class
 *
 * Allow access to an I2C connected MCP23017 16-bit I/O extender chip
 * Example:
 * @code
 *      MCP23017     *par_port;
 * @endcode
 *
 */
class MCP23017
{
  public:
    static bool Init(void);

    /** Reset MCP23017 device to its power-on state
     */
    static void reset(void);

    /** Write a 0/1 value to an output bit
     *
     * @param   value         0 or 1
     * @param   bit_number    bit number range 0 --> 15
     */
    static void write_bit(int value, int bit_number);

    /** Write a masked 16-bit value to the device
     *
     * @param   data    16-bit data value
     * @param   mask    16-bit mask value
     */
    static void write_mask(unsigned short data, unsigned short mask);

    /** Read a 0/1 value from an input bit
     *
     * @param   bit_number    bit number range 0 --> 15
     * @return                0/1 value read
     */
    static int  read_bit(int bit_number);

    /** Read a 16-bit value from the device and apply mask
     *
     * @param   mask    16-bit mask value
     * @return          16-bit data with mask applied
     */
    static int  read_mask(unsigned short mask);

    /** Configure an MCP23017 device
     *
     * @param   dir_config         data direction value (1 = input, 0 = output)
     * @param   pullup_config      100k pullup value (1 = enabled, 0 = disabled)
     * @param   polarity_config    polarity value (1 = flip, 0 = normal)
     */
    static void config(unsigned short dir_config, unsigned short pullup_config, unsigned short polarity_config);

    static void writeRegister(int regAddress, unsigned char  val);
    static void writeRegister(int regAddress, unsigned short val);
    static int  readRegister(int regAddress);

    /*-----------------------------------------------------------------------------
     * pinmode
     * Set units to sequential, bank0 mode
     */
    static void pinMode(int pin, int mode);
    static void digitalWrite(int pin, int val);
    static int  digitalRead(int pin);

    // These provide a more advanced mapping of the chip functionality
    // See the data sheet for more information on what they do

    //Returns a word with the current pin states (ie contents of the GPIO register)
    static unsigned short digitalWordRead(void);

    // Allows you to write a word to the GPIO register
    static void digitalWordWrite(unsigned short w);

    // Sets up the polarity mask that the MCP23017 supports
    // if set to 1, it will flip the actual pin value.
    static void inputPolarityMask(unsigned short mask);

    //Sets which pins are inputs or outputs (1 = input, 0 = output) NB Opposite to arduino's
    //definition for these
    static void inputOutputMask(unsigned short mask);

    // Allows enabling of the internal 100k pullup resisters (1 = enabled, 0 = disabled)
    static void internalPullupMask(unsigned short mask);

    static int read(void);

    static void write(int data);


  private:
    MCP23017() : _i2c(RJ_I2C_BUS) {  }

    static void set_config(PinName sda, PinName scl, int i2cAddress = RJ_IO_EXPANDER_I2C_ADDRESS);
    static std::shared_ptr<MCP23017>& Instance(void);
    static std::shared_ptr<MCP23017> instance;

    I2C              _i2c;
    PinName          _sda, _scl;
    int              _i2cAddress;                        // physical I2C address
    unsigned short   shadow_GPIO, shadow_IODIR, shadow_GPPU, shadow_IPOL;     // Cached copies of the register values
};
