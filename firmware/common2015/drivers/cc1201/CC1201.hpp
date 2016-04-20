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

/**
 * @brief The CC1201 class handles wirelessly sending and receiving data using
 *     the TI CC1201 radio transceiver.
 *
 * The radio is configured via a collection of registers.  These control things
 * such as transceiver frequency, packet error checking, packet length, among
 * MANY other things.
 *
 * The best resource for learning more about the radio and it's SPI interface is
 * the User Guide on TI's website:
 * http://www.ti.com/lit/ug/swru346b/swru346b.pdf
 */
class CC1201 : public CommLink {
public:
    /**
     * Initialize the CC1201 with the given SPI, chip select (inverted), and
     * interrupt line pins.
     *
     * The radio is configured by providing an array of registerSetting_t's
     * which specify (address, value pairs) for configuration registers.
     * Typically the register settings files are exported from TI's SmartRF
     * Studio program.
     *
     * @param regs An array of registereSetting_t values
     * @param len The length of the @regs array
     */
    CC1201(std::shared_ptr<SharedSPI> sharedSPI, PinName nCs, PinName intPin,
           const registerSetting_t* regs, size_t len,
           int rssiOffset = DEFAULT_RSSI_OFFSET);

    /**
     * Transmit data
     *
     * @param buf A buffer containing the data to send
     * @param size The length, in bytes, of @buf
     *
     * @return A status value indicating success/error. See CommLink for info.
     */
    int32_t sendData(const uint8_t* buf, uint8_t size);

    /**
     * Read data from the radio's RX buffer.  This should be called after
     * receiving an interrupt via the radio's interrupt pin.
     *
     * @param buf A pointer to a buffer to write the data into.
     * @return A status value indicating success/error. See CommLink for info.
     */
    int32_t getData(std::vector<uint8_t>* buf);

    void reset();

    int32_t selfTest();

    bool isConnected() const;

    uint8_t mode();

    uint8_t strobe(uint8_t cmd);

    uint8_t readReg(uint16_t addr);
    uint8_t readReg(uint16_t addr, uint8_t* dataOut, uint8_t);

    uint8_t writeReg(uint16_t addr, uint8_t value);
    uint8_t writeReg(uint16_t addr, const uint8_t* data, uint8_t len);

    /// Read the radio's frequency, in MHz
    float freq();

    bool isLocked();

    /// Enable or disable logging of all strobe() commands
    void setDebugEnabled(bool enabled = true) { _debugEnabled = enabled; }
    bool isDebugEnabled() const { return _debugEnabled; }

protected:
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
    float _rssi;

    // In debug mode, all strobe commands are logged at INF2
    // note that this decreases performance, so shouldn't be used normally
    bool _debugEnabled = false;
};

// TODO(justin): remove this
extern CC1201* global_radio;
