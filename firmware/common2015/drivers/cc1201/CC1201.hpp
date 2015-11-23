#pragma once

#include "mbed.h"
#include "rtos.h"
#include "CommLink.hpp"

#define CC1201_DEFAULT_RSSI_OFFSET (-81)


// The config file exported from RF Studio contains an array consisting of these
// structs.
typedef struct {
    uint16_t addr;
    uint8_t value;
} registerSetting_t;



class CC1201 : public CommLink {
public:
    CC1201(PinName mosi, PinName miso, PinName sck, PinName cs,
           PinName intPin, const registerSetting_t* regs, size_t len, int rssiOffset = CC1201_DEFAULT_RSSI_OFFSET);

    virtual ~CC1201();

    virtual int32_t sendData(uint8_t*, uint8_t);

    virtual int32_t getData(uint8_t*, uint8_t*);

    virtual void reset();

    virtual int32_t selfTest();

    virtual bool isConnected();

    void powerOnReset();

    uint8_t mode();

    uint8_t status();

    // TODO: Move any direct register reads/writes & strobes to protected when
    // done testing
    uint8_t strobe(uint8_t cmd);

    uint8_t readReg(uint16_t addr);
    uint8_t readReg(uint16_t addr, uint8_t* dataOut, uint8_t); // TODO: var names?

    uint8_t writeReg(uint16_t addr, uint8_t value);
    uint8_t writeReg(uint16_t addr, const uint8_t* data, uint8_t len);

    void flush_tx();
    void flush_rx();
    void calibrate();
    void update_rssi();
    float rssi();
    uint8_t idle();
    uint8_t rand();
    uint8_t freqUpdate();
    float freq();
    bool isLocked();

protected:
    void set_rssi_offset(int8_t offset);

    /// Sets the radio's register configuration to the values in the given @regs
    /// array.  This array should be exported from SmartRf Studio.
    /// See cfg/readme.md form more info.
    // TODO: return value to indicate success
    void setConfig(const registerSetting_t* regs, size_t len);

private:
    uint8_t status(uint16_t addr);

    uint8_t twos_compliment(uint8_t val);

    uint8_t _lqi;
    uint8_t _chip_version;

    bool _isInit;
    bool _offset_reg_written;
    float _rssi;
};
