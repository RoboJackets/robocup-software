#pragma once

#include "mbed.h"
#include "rtos.h"
#include "CommLink.hpp"

enum ext_flag_t { EXT_FLAG_OFF, EXT_FLAG_ON };

class CC1201 : public CommLink {
public:
    CC1201(){};

    CC1201(PinName mosi, PinName miso, PinName sck, PinName cs,
           PinName intPin = NC, int rssiOffset = DEFAULT_RSSI_OFFSET);

    virtual ~CC1201() { CommLink::cleanup(); }

    virtual int32_t sendData(uint8_t*, uint8_t);

    virtual int32_t getData(uint8_t*, uint8_t*);

    virtual void reset();

    virtual int32_t selfTest();

    virtual bool isConnected();

    uint8_t mode();

    uint8_t status();

    uint8_t strobe(uint8_t);

    uint8_t readReg(uint8_t, ext_flag_t = EXT_FLAG_OFF);

    uint8_t readReg(uint8_t, uint8_t*, uint8_t, ext_flag_t = EXT_FLAG_OFF);

    uint8_t writeReg(uint8_t, uint8_t, ext_flag_t = EXT_FLAG_OFF);

    uint8_t writeReg(uint8_t, uint8_t*, uint8_t, ext_flag_t = EXT_FLAG_OFF);

    float freq();

    bool isLocked();

protected:
    void set_rssi_offset(int8_t offset);

    void flush_tx();

    void flush_rx();

    void calibrate();

    void update_rssi();

    float rssi();

    uint8_t idle();

    uint8_t rand();

    uint8_t freqUpdate();

    static const int DEFAULT_RSSI_OFFSET = -81;

private:
    uint8_t status(uint8_t addr);

    uint8_t readRegExt(uint8_t addr);

    uint8_t readRegExt(uint8_t addr, uint8_t* buffer, uint8_t len);

    uint8_t writeRegExt(uint8_t addr, uint8_t value);

    uint8_t writeRegExt(uint8_t addr, uint8_t* buffer, uint8_t len);

    uint8_t _lqi;
    uint8_t _chip_version;
    bool _isInit;
    bool _offset_reg_written;
    float _rssi;
};
