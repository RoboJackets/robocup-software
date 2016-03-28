#include "CC1201.hpp"

#include "logger.hpp"
#include "assert.hpp"

// check that the address byte doesn't have any non-address bits set
// see "3.2 Access Types" in User Guide
void ASSERT_IS_ADDR(uint16_t addr) {
    ASSERT(addr <= 0x002E || (addr >= 0x2F00 && addr <= 0x2FFF) ||
           (addr >= 0x3E00 && addr <= 0x3EFF) || (addr == 0x003F) ||
           (addr == 0x007F) || (addr == 0x00BF) || (addr == 0x00FF));
}

// TODO(justin): remove this
CC1201* global_radio = nullptr;

CC1201::CC1201(shared_ptr<SharedSPI> sharedSPI, PinName nCs, PinName intPin,
               const registerSetting_t* regs, size_t len, int rssiOffset)
    : CommLink(sharedSPI, nCs, intPin) {
    reset();
    selfTest();

    if (_isInit) {
        // set initial configuration
        setConfig(regs, len);

        writeReg(CC1201_AGC_GAIN_ADJUST, twos_compliment(rssiOffset));

        // start out in RX mode
        strobe(CC1201_STROBE_SRX);

        LOG(INIT, "CC1201 ready!");
        CommLink::ready();
    }
}

int32_t CC1201::sendData(const uint8_t* buf, uint8_t size) {
    // Return if there's no functional radio transceiver - the system will
    // lockup otherwise
    if (!_isInit) return -1;

    if (size != (buf[0] + 1)) {
        LOG(SEVERE,
            "Packet size is inconsistent with expected number of "
            "bytes! %u bytes expected, %u bytes found in buffer.",
            size, buf[0]);
        return COMM_FUNC_BUF_ERR;
    }

    strobe(CC1201_STROBE_SFTX);

    // In order for radio transmission to work, the cc1201 must be first strobed
    // into IDLE, then into TX.  We're not sure why this is the case, but it
    // works.  Many hours were spent reading the data sheet to figure out why
    // the transition through IDLE is necessary, but it remains a mystery.  See
    // the GitHub pull request for more info:
    // https://github.com/RoboJackets/robocup-software/pull/562
    strobe(CC1201_STROBE_SIDLE);

    // Send the data to the CC1201.
    radio_select();
    uint8_t device_state =
        _spi->write(CC1201_TXFIFO | CC1201_BURST | CC1201_WRITE);
    for (uint8_t i = 0; i < size; i++) _spi->write(buf[i]);
    radio_deselect();

    // Enter the TX state.
    if ((device_state & CC1201_STATE_TXFIFO_ERROR) ==
        CC1201_STATE_TXFIFO_ERROR) {
        LOG(WARN, "STATE AT TX ERROR: 0x%02X", device_state);
        // flush the TX buffer & return if the FIFO is in a corrupt state
        flush_tx();

        // set in IDLE mode and strobe back into RX to ensure the states will
        // fall through calibration then return
        idle();
        strobe(CC1201_STROBE_SRX);

        return COMM_DEV_BUF_ERR;
    }

    // Enter TX mode
    strobe(CC1201_STROBE_STX);

    // Wait until radio's TX buffer is emptied
    uint8_t bts = 1;
    do {
        bts = readReg(CC1201_NUM_TXBYTES);
        Thread::wait(2);
    } while (bts != 0);

    // send a NOP to read and log the radio's state
    if (_debugEnabled) strobe(CC1201_STROBE_SNOP);

    return COMM_SUCCESS;
}

int32_t CC1201::getData(std::vector<uint8_t>* buf) {
    // TODO(justin): justify this delay.  It doesn't work without it, but why?
    Thread::wait(5);

    // update frequency offset estimate and get the current state while at it
    uint8_t device_state = freqUpdate();
    uint8_t num_rx_bytes = readReg(CC1201_NUM_RXBYTES);

    if ((device_state & CC1201_STATE_RXFIFO_ERROR) ==
        CC1201_STATE_RXFIFO_ERROR) {
        flush_rx();  // flush RX FIFO buffer and place back into RX state
        strobe(CC1201_STROBE_SRX);

        return COMM_DEV_BUF_ERR;
    }

    if (num_rx_bytes > 0) {
        radio_select();
        _spi->write(CC1201_RXFIFO | CC1201_READ | CC1201_BURST);
        for (int i = 0; i < num_rx_bytes; i++) {
            buf->push_back(_spi->write(CC1201_STROBE_SNOP));
        }
        radio_deselect();

        LOG(INF3, "Bytes in RX buffer: %u\r\nPayload bytes: %u", num_rx_bytes,
            (*buf)[0]);
    } else {
        return COMM_NO_DATA;
    }

    update_rssi();

    // Note: we configured the radio to return to RX mode after a successful RX,
    // so there's no need to explicitly strobe it into RX here.

    return COMM_SUCCESS;
}

uint8_t CC1201::readReg(uint16_t addr) {
    ASSERT_IS_ADDR(addr);

    uint8_t returnVal;

    radio_select();
    if (addr >= CC1201_EXTENDED_ACCESS) {
        _spi->write(CC1201_EXTENDED_ACCESS | CC1201_READ);
        _spi->write(addr & 0xFF);
    } else {
        _spi->write(addr | CC1201_READ);
    }
    returnVal = _spi->write(0x00);
    radio_deselect();

    return returnVal;
}

uint8_t CC1201::readReg(uint16_t addr, uint8_t* buffer, uint8_t len) {
    ASSERT_IS_ADDR(addr);

    uint8_t status_byte;

    radio_select();
    if (addr >= CC1201_EXTENDED_ACCESS) {
        status_byte =
            _spi->write(CC1201_EXTENDED_ACCESS | CC1201_READ | CC1201_BURST);
        _spi->write(addr & 0xFF);
    } else {
        status_byte = _spi->write(addr | CC1201_READ | CC1201_BURST);
    }
    for (uint8_t i = 0; i < len; i++) buffer[i] = _spi->write(0x00);
    radio_deselect();

    return status_byte;
}

uint8_t CC1201::writeReg(uint16_t addr, uint8_t value) {
    ASSERT_IS_ADDR(addr);

    uint8_t status_byte;

    radio_select();
    if (addr >= CC1201_EXTENDED_ACCESS) {
        status_byte = _spi->write(CC1201_EXTENDED_ACCESS | CC1201_WRITE);
        _spi->write(addr & 0xFF);
    } else {
        status_byte = _spi->write(addr);
    }
    _spi->write(value);
    radio_deselect();

    return status_byte;
}

uint8_t CC1201::writeReg(uint16_t addr, const uint8_t* buffer, uint8_t len) {
    ASSERT_IS_ADDR(addr);

    uint8_t status_byte;

    radio_select();
    if (addr >= CC1201_EXTENDED_ACCESS) {
        status_byte =
            _spi->write(CC1201_EXTENDED_ACCESS | CC1201_WRITE | CC1201_BURST);
        _spi->write(addr & 0xFF);  // write lower byte of address
    } else {
        // write lower byte of address
        status_byte = _spi->write(addr | CC1201_WRITE | CC1201_BURST);
    }
    for (uint8_t i = 0; i < len; i++) _spi->write(buffer[i]);
    radio_deselect();

    return status_byte;
}

uint8_t CC1201::strobe(uint8_t addr) {
    if (addr > 0x3d || addr < 0x30) {
        LOG(WARN, "Invalid address: %02X", addr);
        return -1;
    }

    radio_select();
    uint8_t ret = _spi->write(addr);
    radio_deselect();

    // If debug is enabled, we wait for a brief interval, then send a NOP to get
    // the radio's status, then log it to the console
    if (_debugEnabled) {
        // wait a bit to allow the previous strobe to take effect
        // TODO: how long should this delay actually be?
        int delay = 2;
        Thread::wait(delay);

        radio_select();
        uint8_t ret2 = _spi->write(CC1201_STROBE_SNOP);
        radio_deselect();

        const char* strobe_names[] = {
            "RES",     // 0x30
            "FSTXON",  // 0x31
            "XOFF",    // 0x32
            "CAL",     // 0x33
            "RX",      // 0x34
            "TX",      // 0x35
            "IDLE",    // 0x36
            "AFC",     // 0x37
            "WOR",     // 0x38
            "PWD",     // 0x39
            "FRX",     // 0x3a
            "FTX",     // 0x3b
            "WORRST",  // 0x3c
            "NOP"      // 0x3d
        };

        const char* state_names[] = {"IDLE",        "RX",         "TX",
                                     "FSTXON",      "CALIB",      "SETTLE",
                                     "RX_FIFO_ERR", "TX_FIFO_ERR"};

        // The status byte returned from strobe() contains from (msb to lsb):
        // * 1 bit - chip ready (0 indicates xosc is stable)
        // * 3 bits - state
        // * 4 bits - unused
        int rdy_n = ret & (1 << 7);
        int state = (ret >> 4) & 7;
        int rdy2_n = ret2 & (1 << 7);
        int state2 = (ret2 >> 4) & 7;
        LOG(INF2,
            "strobe '%s' sent, status = {rdy_n: %d, state: %s}, "
            "after %dms = {rdy_n: %d, state: %s}",
            strobe_names[addr - 0x30], rdy_n, state_names[state], delay, rdy2_n,
            state_names[state2]);
    }

    return ret;
}

uint8_t CC1201::mode() { return 0x1F & readReg(CC1201_MARCSTATE); }

void CC1201::reset() {
    idle();
    radio_select();
    _spi->write(CC1201_STROBE_SRES);
    radio_deselect();

    // Wait up to 300ms for the radio to do anything. Don't block everything
    // else if it doesn't startup correctly
    for (size_t i = 0; i < 300; i++) {
        if (~(idle()) & 0x80)  // Chip is ready when status byte's MSB is 0
            break;
        else
            Thread::wait(1);
    }

    _isInit = false;
}

int32_t CC1201::selfTest() {
    if (_isInit) return 0;

    _chip_version = readReg(CC1201_PARTNUMBER);

    if (_chip_version != CC1201_EXPECTED_PARTNUMBER) {
        LOG(FATAL,
            "CC1201 part number error:\r\n"
            "    Found:\t0x%02X (expected 0x%02X)",
            _chip_version, CC1201_EXPECTED_PARTNUMBER);

        return -1;
    } else {
        _isInit = true;
        idle();
        return 0;
    }
}

bool CC1201::isConnected() const { return _isInit; }

void CC1201::flush_tx() {
    idle();
    size_t bytes = readReg(CC1201_NUM_TXBYTES);
    strobe(CC1201_STROBE_SFTX);
    LOG(WARN, "%u bytes flushed from TX FIFO buffer.", bytes);
}

void CC1201::flush_rx() {
    idle();
    size_t bytes = readReg(CC1201_NUM_RXBYTES);
    strobe(CC1201_STROBE_SFRX);
    LOG(WARN, "%u bytes flushed from RX FIFO buffer.", bytes);
}

void CC1201::calibrate() {
    idle();
    strobe(CC1201_STROBE_SCAL);
}

void CC1201::update_rssi() {
    // Only use the top MSB for simplicity. 1 dBm resolution.
    uint8_t offset = readReg(CC1201_RSSI1);
    _rssi = static_cast<float>((int8_t)twos_compliment(offset));

    LOG(INF3, "RSSI is from device.");

    LOG(INF3, "RSSI Register Val: 0x%02X", _rssi);
}

float CC1201::rssi() { return _rssi; }

uint8_t CC1201::idle() {
    uint8_t status_byte = strobe(CC1201_STROBE_SIDLE);

    if (!_isInit) return status_byte;

    // Wait up to 300ms for the radio to do become ready
    for (size_t i = 0; i < 300; i++) {
        if (mode() == 0x01)  // chip is in idle state
            break;
        else
            Thread::wait(1);
    }

    return status_byte;
}

uint8_t CC1201::freqUpdate() { return strobe(CC1201_STROBE_SAFC); }

float CC1201::freq() {
    freqUpdate();

    // read the 5 frequency-related bytes in order:
    // [FREQOFF1, FREQOFF0, FREQ2, FREQ1, FREQ0]
    uint8_t buf[5];
    readReg(CC1201_FREQOFF1, buf, 5);

    // concatenate the two FREQOFF bytes and convert from 2's complement
    const uint16_t freq_offset = ~((buf[0] << 8) | buf[1]) + 1;

    uint32_t freq_base = (buf[2] << 16) | (buf[3] << 8) | buf[4];

    // crystal oscillator is 40MHz
    constexpr float f_xosc = 40000000;

    float f_vco = (((float)freq_base) * f_xosc / powf(2, 16)) +
                  (((float)freq_offset) * f_xosc / powf(2, 18));

    // LO divider is 4 for our selected band, which is 820MHz - 960MHz
    float lo_divider = 4;

    float freq = f_vco / lo_divider / 1000000;

    LOG(INF2, "Operating Frequency: %3.2f MHz", freq);

    return freq;
}

bool CC1201::isLocked() {
    // This is only valid in RX, TX, & FSTXON
    return readReg(CC1201_FSCAL_CTRL) & 0x01;
}

void CC1201::setConfig(const registerSetting_t* regs, size_t len) {
    if (!regs) return;

    for (size_t i = 0; i < len; ++i) {
        writeReg(regs[i].addr, regs[i].value);
    }
}
