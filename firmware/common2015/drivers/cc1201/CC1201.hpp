#pragma once

#include "mbed.h"
#include "rtos.h"
#include "CommLink.hpp"

#define CC1201_DEFAULT_RSSI_OFFSET (-81)

enum ext_flag_t { EXT_FLAG_OFF, EXT_FLAG_ON };

class CC1201 : public CommLink {
public:
    CC1201();

    CC1201(PinName mosi, PinName miso, PinName sck, PinName cs,
           PinName intPin = NC, int rssiOffset = CC1201_DEFAULT_RSSI_OFFSET);

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
    uint8_t strobe(uint8_t);

    uint8_t readReg(uint8_t, ext_flag_t = EXT_FLAG_OFF);

    uint8_t readReg(uint8_t, uint8_t*, uint8_t, ext_flag_t = EXT_FLAG_OFF);

    uint8_t writeReg(uint8_t, uint8_t, ext_flag_t = EXT_FLAG_OFF);

    uint8_t writeReg(uint8_t, uint8_t*, uint8_t, ext_flag_t = EXT_FLAG_OFF);

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

private:
    uint8_t status(uint8_t addr);

    uint8_t readRegExt(uint8_t addr);

    uint8_t readRegExt(uint8_t addr, uint8_t* buffer, uint8_t len);

    uint8_t writeRegExt(uint8_t addr, uint8_t value);

    uint8_t writeRegExt(uint8_t addr, uint8_t* buffer, uint8_t len);

    uint8_t twos_compliment(uint8_t val);

    uint8_t _lqi;
    uint8_t _chip_version;

    bool _isInit;
    bool _offset_reg_written;
    float _rssi;
};
