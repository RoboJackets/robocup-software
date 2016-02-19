#pragma once

#include "mbed.h"
#include "rtos.h"
#include "CommLink.hpp"
#include "ti/defines.hpp"

// The config file exported from RF Studio contains an array consisting of these
// structs.
typedef struct {
    uint16_t addr;
    uint8_t value;
} registerSetting_t;

class CC1201 : public CommLink {
public:
    CC1201(PinName mosi, PinName miso, PinName sck, PinName cs, PinName intPin,
           const registerSetting_t* regs, size_t len,
           int rssiOffset = DEFAULT_RSSI_OFFSET);

    int32_t sendData(uint8_t*, uint8_t);

    int32_t getData(uint8_t*, uint8_t*);

    void reset();

    int32_t selfTest();

    bool isConnected() const;

    uint8_t mode();

    uint8_t status(uint8_t strobe = CC1201_STROBE_SNOP);

    // TODO: Move any direct register reads/writes & strobes to protected when
    // done testing
    uint8_t strobe(uint8_t cmd);

    uint8_t readReg(uint16_t addr);
    uint8_t readReg(uint16_t addr, uint8_t* dataOut,
                    uint8_t);  // TODO: var names?

    uint8_t writeReg(uint16_t addr, uint8_t value);
    uint8_t writeReg(uint16_t addr, const uint8_t* data, uint8_t len);

    float freq();

    bool isLocked();

    /// Enable or disable logging of all strobe() commands
    void setDebugEnabled(bool enabled = true) { _debugEnabled = enabled; }
    bool isDebugEnabled() const { return _debugEnabled; }

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

    /// Sets the radio's register configuration to the values in the given @regs
    /// array.  This array should be exported from SmartRf Studio.
    /// See cfg/readme.md form more info.
    // TODO: return value to indicate success
    void setConfig(const registerSetting_t* regs, size_t len);

private:
    uint8_t _lqi;
    uint8_t _chip_version;
    bool _isInit;
    bool _offset_reg_written;
    float _rssi;

    // In debug mode, all strobe commands are logged at INF2
    // note that this decreases performance, so shouldn't be used normally
    bool _debugEnabled = false;
};

// TODO(justin): remove this
extern CC1201* global_radio;
