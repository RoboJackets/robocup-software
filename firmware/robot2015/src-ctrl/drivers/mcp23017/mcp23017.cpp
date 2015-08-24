#include "mcp23017.hpp"

#include <mbed.h>
#include <logger.hpp>


// Declaration for the pointer to the global object
shared_ptr<MCP23017> MCP23017::instance;


void MCP23017::set_config(PinName sda, PinName scl, int i2cAddress)
{
    instance->_sda = sda;
    instance->_scl = scl;
    instance->_i2cAddress = i2cAddress;

    LOG(OK,
        "IO Expander I2C Address:\t0x%02X",
        instance->_i2cAddress
       );
}

shared_ptr<MCP23017>& MCP23017::Instance(void)
{
    if (instance.get() == nullptr)
        instance.reset(new MCP23017);

    return instance;
}

bool MCP23017::Init(void)
{
    auto instance = Instance();

    set_config(RJ_I2C_BUS);
    reset();
    config(0x00FF, 0x0000, 0xFF00);

    LOG(OK, "MCP23017 initialized");

    return true;
}

/*-----------------------------------------------------------------------------
 * reset
 * Set configuration (IOCON) and direction(IODIR) registers to initial state
 */
void MCP23017::reset(void)
{
    // First make sure that the device is in BANK=0 mode
    writeRegister(0x05, (unsigned char)0x04);

    // Set the shadow registers to power-on state
    inputOutputMask(0xFFFF);

    // set all other registers to zero (last of 10 registers is OLAT)
    for (int reg_addr = 2; reg_addr <= OLAT; reg_addr += 2)
        writeRegister(reg_addr, (unsigned short)0x0000);

    instance->shadow_GPIO  = 0;
    instance->shadow_GPPU  = 0;
    instance->shadow_IPOL  = 0;
}

/*-----------------------------------------------------------------------------
 * writeRegister
 * write a byte
 */
void MCP23017::writeRegister(int regAddress, unsigned char data)
{
    char  buffer[2];
    buffer[0] = regAddress;
    buffer[1] = data;

    int ret = instance->_i2c.write(instance->_i2cAddress, buffer, 2);

    if (!ret)
        LOG(SEVERE, "No ACK received on I2C bus.\r\n    Slave Address:\t0x%02X", instance->_i2cAddress);
}

/*----------------------------------------------------------------------------
 * write Register
 * write two bytes
 */
void MCP23017::writeRegister(int regAddress, unsigned short data)
{
    char  buffer[3];
    buffer[0] = regAddress;
    buffer[1] = data & 0xFF;
    buffer[2] = (data >> 8) & 0xFF;

    int ret = instance->_i2c.write(instance->_i2cAddress, buffer, 3);

    if (!ret)
        LOG(SEVERE, "No ACK received on I2C bus.\r\n    Slave Address:\t0x%02X", instance->_i2cAddress);
}

/*-----------------------------------------------------------------------------
 * readRegister
 */
int MCP23017::readRegister(int regAddress)
{
    char buffer[2];
    instance->_i2c.write(regAddress);
    instance->_i2c.read(instance->_i2cAddress, buffer, 2);

    return (int)(buffer[0] | (buffer[1] << 8));
}

/*-----------------------------------------------------------------------------
 * write_bit
 * Write a 1/0 to a single bit of the 16-bit port
 */
void MCP23017::write_bit(int value, int bit_number)
{
    if (value == 0) {
        instance->shadow_GPIO &= ~(1 << bit_number);
    } else {
        instance->shadow_GPIO |= 1 << bit_number;
    }

    digitalWordWrite(instance->shadow_GPIO);
}

/*-----------------------------------------------------------------------------
 * Write a combination of bits to the 16-bit port
 */
void MCP23017::write_mask(unsigned short data, unsigned short mask)
{
    instance->shadow_GPIO = (instance->shadow_GPIO & ~mask) | data;
    digitalWordWrite(instance->shadow_GPIO);
}

/*-----------------------------------------------------------------------------
 * read_bit
 * Read a single bit from the 16-bit port
 */
int  MCP23017::read_bit(int bit_number)
{
    instance->shadow_GPIO = digitalWordRead();

    LOG(INF2,
        "Read an I/O pin bit:"
        "    Bit:\t%u\r\n"
        "    State:\t%s",
        bit_number,
        ((instance->shadow_GPIO >> bit_number) & 0x0001) ? "ON" : "OFF"
       );

    return  ((instance->shadow_GPIO >> bit_number) & 0x0001);
}

/*-----------------------------------------------------------------------------
 * read_mask
 */
int  MCP23017::read_mask(unsigned short mask)
{
    return readRegister(GPIO) & mask;
}

/*-----------------------------------------------------------------------------
 * Config
 * set direction and pull-up registers
 */
void MCP23017::config(unsigned short dir_config, unsigned short pullup_config,  unsigned short polarity_config)
{
    inputOutputMask(dir_config);
    internalPullupMask(pullup_config);
    inputPolarityMask(polarity_config);

    LOG(INF2,
        "IO Expander Configuration:\r\n"
        "    IODIR:\t0x%04X\r\n"
        "    GPPU:\t0x%04X\r\n"
        "    IPOL:\t0x%04X",
        instance->shadow_IODIR,
        instance->shadow_GPPU,
        instance->shadow_IPOL
       );
}

/*-----------------------------------------------------------------------------
 * pinMode
 */
void MCP23017::pinMode(int pin, int mode)
{
    if (DIR_INPUT) {
        instance->shadow_IODIR |= 1 << pin;
    } else {
        instance->shadow_IODIR &= ~(1 << pin);
    }

    inputOutputMask(instance->shadow_IODIR);
}

/*-----------------------------------------------------------------------------
 * digitalRead
 */
int MCP23017::digitalRead(int pin)
{
    return ((readRegister(GPIO) & (1 << pin)) ? 1 : 0);
}

/*-----------------------------------------------------------------------------
 * digitalWrite
 */
void MCP23017::digitalWrite(int pin, int val)
{
    //If this pin is an INPUT pin, a write here will
    //enable the internal pullup
    //otherwise, it will set the OUTPUT voltage
    //as appropriate.
    bool isOutput = !(instance->shadow_IODIR & 1 << pin);

    if (isOutput) {
        //This is an output pin so just write the value
        if (val) instance->shadow_GPIO |= 1 << pin;
        else instance->shadow_GPIO &= ~(1 << pin);

        digitalWordWrite(instance->shadow_GPIO);
    } else {
        //This is an input pin, so we need to enable the pullup
        if (val) {
            instance->shadow_GPPU |= 1 << pin;
        } else {
            instance->shadow_GPPU &= ~(1 << pin);
        }

        internalPullupMask(instance->shadow_GPPU);
    }
}

/*-----------------------------------------------------------------------------
 * digitalWordRead
 */
unsigned short MCP23017::digitalWordRead(void)
{
    return readRegister(GPIO);
}

/*-----------------------------------------------------------------------------
 * digitalWordWrite
 */
void MCP23017::digitalWordWrite(unsigned short w)
{
    writeRegister(GPIO, w);
}

/*-----------------------------------------------------------------------------
 * inputPolarityMask
 */
void MCP23017::inputPolarityMask(unsigned short mask)
{
    instance->shadow_IPOL = mask;
    writeRegister(IPOL, (unsigned short)instance->shadow_IPOL);
}

/*-----------------------------------------------------------------------------
 * inputoutputMask
 */
void MCP23017::inputOutputMask(unsigned short mask)
{
    instance->shadow_IODIR = mask;
    writeRegister(IODIR, (unsigned short)instance->shadow_IODIR);
}

/*-----------------------------------------------------------------------------
 * internalPullupMask
 */
void MCP23017::internalPullupMask(unsigned short mask)
{
    instance->shadow_GPPU = mask;
    writeRegister(GPPU, (unsigned short)instance->shadow_GPPU);
}

