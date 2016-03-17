#include "mcp23017.hpp"

#include <mbed.h>
#include <logger.hpp>

MCP23017::MCP23017(PinName sda, PinName scl, int i2cAddress)
    : _i2c(sda, scl), _i2cAddress(i2cAddress) {
    _i2c.frequency(400000);
    reset();
    config(0x00FF, 0x0000, 0x0000);

    LOG(OK, "MCP23017 initialized");
}

void MCP23017::reset() {
    // Set all pins to input mode (via IODIR register)
    inputOutputMask(0xFFFF);

    // set all other registers to zero (last of 10 registers is OLAT)
    for (int reg_addr = 2; reg_addr <= OLAT; reg_addr += 2)
        writeRegister(static_cast<MCP23017::Register>(reg_addr), 0x0000);

    // reset cached values
    _cachedGPIO = 0;
    _cachedGPPU = 0;
    _cachedIPOL = 0;
}

void MCP23017::writeRegister(MCP23017::Register regAddress, uint8_t data) {
    char buffer[] = {regAddress, data};
    _i2c.write(_i2cAddress, buffer, sizeof(buffer));
}

uint16_t MCP23017::readRegister(MCP23017::Register regAddress) {
    char buffer[2];
    _i2c.write(regAddress);
    _i2c.read(_i2cAddress, buffer, 2);

    return (uint16_t)(buffer[0] | (buffer[1] << 8));
}

void MCP23017::writeBit(int value, int bit_number) {
    if (value == 0) {
        _cachedGPIO &= ~(1 << bit_number);
    } else {
        _cachedGPIO |= 1 << bit_number;
    }

    digitalWordWrite(_cachedGPIO);
}

void MCP23017::writeMask(uint16_t data, uint16_t mask) {
    _cachedGPIO = (_cachedGPIO & ~mask) | data;
    digitalWordWrite(_cachedGPIO);
}

uint8_t MCP23017::readBit(int bit_number) {
    _cachedGPIO = digitalWordRead();

    LOG(INF2,
        "Read an I/O pin bit:"
        "    Bit:\t%u\r\n"
        "    State:\t%s",
        bit_number, ((_cachedGPIO >> bit_number) & 0x0001) ? "ON" : "OFF");

    return ((_cachedGPIO >> bit_number) & 0x0001);
}

int MCP23017::readMask(uint16_t mask) { return readRegister(GPIO) & mask; }

void MCP23017::config(uint16_t dir_config, uint16_t pullup_config,
                      uint16_t polarity_config) {
    inputOutputMask(dir_config);
    internalPullupMask(pullup_config);
    inputPolarityMask(polarity_config);

    LOG(INF2,
        "IO Expander Configuration:\r\n"
        "    IODIR:\t0x%04X\r\n"
        "    GPPU:\t0x%04X\r\n"
        "    IPOL:\t0x%04X",
        _cachedIODIR, _cachedGPPU, _cachedIPOL);
}

void MCP23017::pinMode(int pin, PinMode mode) {
    if (mode == DIR_INPUT) {
        _cachedIODIR |= 1 << pin;
    } else {
        _cachedIODIR &= ~(1 << pin);
    }

    inputOutputMask(_cachedIODIR);
}

int MCP23017::digitalRead(int pin) {
    _cachedGPIO = readRegister(GPIO);
    return ((_cachedGPIO & (1 << pin)) ? 1 : 0);
}

void MCP23017::digitalWrite(int pin, int val) {
    // If this pin is an INPUT pin, a write here will
    // enable the internal pullup
    // otherwise, it will set the OUTPUT voltage
    // as appropriate.
    bool isOutput = !(_cachedIODIR & 1 << pin);

    if (isOutput) {
        // This is an output pin so just write the value
        if (val)
            _cachedGPIO |= 1 << pin;
        else
            _cachedGPIO &= ~(1 << pin);

        digitalWordWrite(_cachedGPIO);
    } else {
        // This is an input pin, so we need to enable the pullup
        if (val) {
            _cachedGPPU |= 1 << pin;
        } else {
            _cachedGPPU &= ~(1 << pin);
        }

        internalPullupMask(_cachedGPPU);
    }
}

uint16_t MCP23017::digitalWordRead() {
    _cachedGPIO = readRegister(GPIO);
    return _cachedGPIO;
}

void MCP23017::digitalWordWrite(uint16_t w) {
    _cachedGPIO = w;
    writeRegister(GPIO, w);
}

void MCP23017::inputPolarityMask(uint16_t mask) {
    _cachedIPOL = mask;
    writeRegister(IPOL, (uint16_t)_cachedIPOL);
}

void MCP23017::inputOutputMask(uint16_t mask) {
    _cachedIODIR = mask;
    writeRegister(IODIR, (uint16_t)_cachedIODIR);
}

void MCP23017::internalPullupMask(uint16_t mask) {
    _cachedGPPU = mask;
    writeRegister(GPPU, (uint16_t)_cachedGPPU);
}
