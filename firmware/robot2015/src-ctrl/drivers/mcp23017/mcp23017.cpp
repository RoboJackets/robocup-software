/*  MCP23017 library for Arduino
    Copyright (C) 2009 David Pye    <davidmpye@gmail.com
    Modified for use on the MBED ARM platform

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "mcp23017.hpp"
#include "mbed.h"


I2C*            _i2c;
PinName         _sda, _scl;
int             _i2cAddress;
unsigned short  shadow_GPIO, shadow_IODIR, shadow_GPPU, shadow_IPOL;


union {
    uint8_t  value8[2];
    uint16_t value16;
} tmp_data;

/*-----------------------------------------------------------------------------
 *
 */
MCP23017::MCP23017(PinName sda, PinName scl, int i2cAddress)
{
    _sda = sda;
    _scl = scl;
    _i2cAddress = i2cAddress;

    init();

    reset();                                  // initialise chip to power-on condition
}

MCP23017::~MCP23017(void)
{
    if (_i2c)
        delete _i2c;
}

bool MCP23017::init(void)
{
    _i2c = new I2C(_sda, _scl);

    return true;
}

/*-----------------------------------------------------------------------------
 * reset
 * Set configuration (IOCON) and direction(IODIR) registers to initial state
 */
void MCP23017::reset()
{
    // First make sure that the device is in BANK=0 mode
    writeRegister(0x05, (unsigned char)0x00);

    // set direction registers to inputs
    writeRegister(IODIR, (unsigned short)0xFFFF);

    // set all other registers to zero (last of 10 registers is OLAT)
    for (int reg_addr = 2 ; reg_addr <= OLAT ; reg_addr += 2) {
        writeRegister(reg_addr, (unsigned short)0x0000);
    }

    // Set the shadow registers to power-on state
    shadow_IODIR = 0xFFFF;
    shadow_GPIO  = 0;
    shadow_GPPU  = 0;
    shadow_IPOL  = 0;
}

/*-----------------------------------------------------------------------------
 * write_bit
 * Write a 1/0 to a single bit of the 16-bit port
 */
void MCP23017::write_bit(int value, int bit_number)
{
    if (value == 0) {
        shadow_GPIO &= ~(1 << bit_number);
    } else {
        shadow_GPIO |= 1 << bit_number;
    }

    writeRegister(GPIO, (unsigned short)shadow_GPIO);
}

/*-----------------------------------------------------------------------------
 * Write a combination of bits to the 16-bit port
 */
void MCP23017::write_mask(unsigned short data, unsigned short mask)
{
    shadow_GPIO = (shadow_GPIO & ~mask) | data;
    writeRegister(GPIO, (unsigned short)shadow_GPIO);
}

/*-----------------------------------------------------------------------------
 * read_bit
 * Read a single bit from the 16-bit port
 */
int  MCP23017::read_bit(int bit_number)
{
    shadow_GPIO = readRegister(GPIO);
    return  ((shadow_GPIO >> bit_number) & 0x0001);
}

/*-----------------------------------------------------------------------------
 * read_mask
 */
int  MCP23017::read_mask(unsigned short mask)
{
    shadow_GPIO = readRegister(GPIO);
    return (shadow_GPIO & mask);
}

/*-----------------------------------------------------------------------------
 * Config
 * set direction and pull-up registers
 */
void MCP23017::config(unsigned short dir_config, unsigned short pullup_config,  unsigned short polarity_config)
{
    shadow_IODIR = dir_config;
    writeRegister(IODIR, (unsigned short)shadow_IODIR);

    shadow_GPPU = pullup_config;
    writeRegister(GPPU, (unsigned short)shadow_GPPU);

    shadow_IPOL = polarity_config;
    writeRegister(IPOL, (unsigned short)shadow_IPOL);
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
    _i2c->write(_i2cAddress, buffer, 2);
}

/*----------------------------------------------------------------------------
 * write Register
 * write two bytes
 */
void MCP23017::writeRegister(int regAddress, unsigned short data)
{
    char  buffer[3];

    buffer[0] = regAddress;
    tmp_data.value16 = data;
    buffer[1] = tmp_data.value8[0];
    buffer[2] = tmp_data.value8[1];

    _i2c->write(_i2cAddress, buffer, 3);
}

/*-----------------------------------------------------------------------------
 * readRegister
 */
int MCP23017::readRegister(int regAddress)
{
    char buffer[2];

    buffer[0] = regAddress;
    _i2c->write(_i2cAddress, buffer, 1);
    _i2c->read(_i2cAddress, buffer, 2);

    return ((int)(buffer[0] + (buffer[1] << 8)));
}

/*-----------------------------------------------------------------------------
 * pinMode
 */
void MCP23017::pinMode(int pin, int mode)
{
    if (DIR_INPUT) {
        shadow_IODIR |= 1 << pin;
    } else {
        shadow_IODIR &= ~(1 << pin);
    }

    writeRegister(IODIR, (unsigned short)shadow_IODIR);
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
    bool isOutput = !(shadow_IODIR & 1 << pin);

    if (isOutput) {
        //This is an output pin so just write the value
        if (val) shadow_GPIO |= 1 << pin;
        else shadow_GPIO &= ~(1 << pin);

        writeRegister(GPIO, (unsigned short)shadow_GPIO);
    } else {
        //This is an input pin, so we need to enable the pullup
        if (val) {
            shadow_GPPU |= 1 << pin;
        } else {
            shadow_GPPU &= ~(1 << pin);
        }

        writeRegister(GPPU, (unsigned short)shadow_GPPU);
    }
}

/*-----------------------------------------------------------------------------
 * digitalWordRead
 */
unsigned short MCP23017::digitalWordRead()
{
    shadow_GPIO = readRegister(GPIO);
    return shadow_GPIO;
}

/*-----------------------------------------------------------------------------
 * digitalWordWrite
 */
void MCP23017::digitalWordWrite(unsigned short w)
{
    shadow_GPIO = w;
    writeRegister(GPIO, (unsigned short)shadow_GPIO);
}

/*-----------------------------------------------------------------------------
 * inputPolarityMask
 */
void MCP23017::inputPolarityMask(unsigned short mask)
{
    writeRegister(IPOL, mask);
}

/*-----------------------------------------------------------------------------
 * inputoutputMask
 */
void MCP23017::inputOutputMask(unsigned short mask)
{
    shadow_IODIR = mask;
    writeRegister(IODIR, (unsigned short)shadow_IODIR);
}

/*-----------------------------------------------------------------------------
 * internalPullupMask
 */
void MCP23017::internalPullupMask(unsigned short mask)
{
    shadow_GPPU = mask;
    writeRegister(GPPU, (unsigned short)shadow_GPPU);
}

