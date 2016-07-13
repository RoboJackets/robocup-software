#include "CC1201.hpp"

#include "assert.hpp"
#include "logger.hpp"

// clang-format off
const char* CC1201_STATE_NAMES[] = {
    "IDLE",
    "RX",
    "TX",
    "FSTXON",
    "CALIB",
    "SETTLE",
    "RX_FIFO_ERR",
    "TX_FIFO_ERR"
};
// clang-format on

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

        setChannel(0);

        // start out in RX mode
        strobe(CC1201_STROBE_SRX);

        LOG(INIT, "CC1201 ready!");
        CommLink::ready();
    }
}

int32_t CC1201::sendPacket(const rtp::packet* pkt) {
    // Return if there's no functional radio transceiver - the system will
    // lockup otherwise
    if (!_isInit) return COMM_FAILURE;

    // In order for radio transmission to work, the cc1201 must be first strobed
    // into IDLE, then into TX.  We're not sure why this is the case, but it
    // works.  Many hours were spent reading the data sheet to figure out why
    // the transition through IDLE is necessary, but it remains a mystery.  See
    // the GitHub pull request for more info:
    // https://github.com/RoboJackets/robocup-software/pull/562
    strobe(CC1201_STROBE_SIDLE);

    // Send the data to the CC1201.
    chipSelect();
    uint8_t device_state =
        _spi->write(CC1201_TXFIFO | CC1201_BURST | CC1201_WRITE);
    _spi->write(pkt->size());  // write size byte first
    uint8_t* headerData = (uint8_t*)&pkt->header;
    for (size_t i = 0; i < sizeof(pkt->header); ++i) _spi->write(headerData[i]);
    for (uint8_t byte : pkt->payload) _spi->write(byte);
    chipDeselect();

    // Enter the TX state.
    if ((device_state & CC1201_STATE_TXFIFO_ERROR) ==
        CC1201_STATE_TXFIFO_ERROR) {
        // flush the TX buffer & return if the FIFO is in a corrupt state
        strobe(CC1201_STROBE_SIDLE);
        strobe(CC1201_STROBE_SFTX);
        strobe(CC1201_STROBE_SRX);

        return COMM_DEV_BUF_ERR;
    }

    // Enter TX mode
    strobe(CC1201_STROBE_STX);

    // Wait until radio's TX buffer is emptied
    // uint8_t bts = 1;
    // do {
    //     bts = readReg(CC1201_NUM_TXBYTES);
    //     Thread::wait(2);
    // } while (bts != 0);

    // send a NOP to read and log the radio's state
    if (_debugEnabled) strobe(CC1201_STROBE_SNOP);

    return COMM_SUCCESS;
}

int32_t CC1201::getData(std::vector<uint8_t>* buf) {
    uint8_t num_rx_bytes = readReg(CC1201_NUM_RXBYTES);
    uint8_t device_state = strobe(CC1201_STROBE_SNOP);

    if ((device_state & CC1201_STATE_RXFIFO_ERROR) ==
        CC1201_STATE_RXFIFO_ERROR) {
        flush_rx();  // flush RX FIFO buffer and place back into RX state
        strobe(CC1201_STROBE_SRX);

        return COMM_DEV_BUF_ERR;
    }

    if (num_rx_bytes > 0) {
        chipSelect();
        _spi->write(CC1201_RXFIFO | CC1201_READ | CC1201_BURST);
        size_t size_byte = _spi->write(CC1201_STROBE_SNOP);

        if (size_byte > num_rx_bytes) {
            // the size byte isn't right
            chipDeselect();
            LOG(WARN, "Invalid size byte: %u, rx byte count reg: %u", size_byte,
                num_rx_bytes);
            strobe(CC1201_STROBE_SIDLE);
            strobe(CC1201_STROBE_SFRX);
            strobe(CC1201_STROBE_SRX);
            return COMM_DEV_BUF_ERR;
        }
        for (uint8_t i = 0; i < size_byte; i++) {
            buf->push_back(_spi->write(CC1201_STROBE_SNOP));
        }
        chipDeselect();
        strobe(CC1201_STROBE_SFRX);

        LOG(INF3, "Bytes in RX buffer: %u, size_byte: %u", num_rx_bytes,
            size_byte);
    } else {
        // flush rx
        strobe(CC1201_STROBE_SIDLE);
        strobe(CC1201_STROBE_SFRX);
        strobe(CC1201_STROBE_SRX);

        return COMM_NO_DATA;
    }

    update_rssi();

    // Note: we configured the radio to return to RX mode after a successful RX,
    // so there's no need to explicitly strobe it into RX here.

    return COMM_SUCCESS;
}

uint8_t CC1201::setAddress(uint8_t addr) {
    return writeReg(CC1201_DEV_ADDR, addr);
}

void CC1201::setChannel(uint8_t chanNumber) {
    // note: the values for @base and @spacing came from changing the frequency
    // in SmartRF studio and seeing how the FREQ{0,1,2} registers changed. If
    // other frequency-related config values are changed, these values will need
    // to be changed.
    const uint32_t base = 5924454;
    const uint32_t spacing = 13107;
    uint32_t freq = base + spacing * chanNumber;

    if (freq > 0xFFFFFF) {
        LOG(SEVERE,
            "Attempt to set radio to invalid channel, setting back to channel "
            "0");
        freq = base;
    }

    writeReg(CC1201_FREQ2, (freq >> 16) & 0xFF);
    writeReg(CC1201_FREQ1, (freq >> 8) & 0xFF);
    writeReg(CC1201_FREQ0, freq & 0xFF);

    // reset so changes take effect
    strobe(CC1201_STROBE_SIDLE);
    strobe(CC1201_STROBE_SRX);
}

uint8_t CC1201::readReg(uint16_t addr) {
    ASSERT_IS_ADDR(addr);

    uint8_t returnVal;

    chipSelect();
    if (addr >= CC1201_EXTENDED_ACCESS) {
        _spi->write(CC1201_EXTENDED_ACCESS | CC1201_READ);
        _spi->write(addr & 0xFF);
    } else {
        _spi->write(addr | CC1201_READ);
    }
    returnVal = _spi->write(0x00);
    chipDeselect();

    return returnVal;
}

uint8_t CC1201::readReg(uint16_t addr, uint8_t* buffer, uint8_t len) {
    ASSERT_IS_ADDR(addr);

    uint8_t status_byte;

    chipSelect();
    if (addr >= CC1201_EXTENDED_ACCESS) {
        status_byte =
            _spi->write(CC1201_EXTENDED_ACCESS | CC1201_READ | CC1201_BURST);
        _spi->write(addr & 0xFF);
    } else {
        status_byte = _spi->write(addr | CC1201_READ | CC1201_BURST);
    }
    for (uint8_t i = 0; i < len; i++) buffer[i] = _spi->write(0x00);
    chipDeselect();

    return status_byte;
}

uint8_t CC1201::writeReg(uint16_t addr, uint8_t value) {
    ASSERT_IS_ADDR(addr);

    uint8_t status_byte;

    chipSelect();
    if (addr >= CC1201_EXTENDED_ACCESS) {
        status_byte = _spi->write(CC1201_EXTENDED_ACCESS | CC1201_WRITE);
        _spi->write(addr & 0xFF);
    } else {
        status_byte = _spi->write(addr);
    }
    _spi->write(value);
    chipDeselect();

    return status_byte;
}

uint8_t CC1201::writeReg(uint16_t addr, const uint8_t* buffer, uint8_t len) {
    ASSERT_IS_ADDR(addr);

    uint8_t status_byte;

    chipSelect();
    if (addr >= CC1201_EXTENDED_ACCESS) {
        status_byte =
            _spi->write(CC1201_EXTENDED_ACCESS | CC1201_WRITE | CC1201_BURST);
        _spi->write(addr & 0xFF);  // write lower byte of address
    } else {
        // write lower byte of address
        status_byte = _spi->write(addr | CC1201_WRITE | CC1201_BURST);
    }
    for (uint8_t i = 0; i < len; i++) _spi->write(buffer[i]);
    chipDeselect();

    return status_byte;
}

void CC1201::printDebugInfo() {
    uint8_t stateByte = strobe(CC1201_STROBE_SNOP);
    bool ready = !(stateByte & 0x80);
    uint8_t state = (stateByte >> 4) & 7;

    printf("Radio Status:\r\n  ready: %u, state: %s, int pin: %u\r\n", ready,
           CC1201_STATE_NAMES[state], _int_in == 1);
}

uint8_t CC1201::strobe(uint8_t addr) {
    if (addr > 0x3d || addr < 0x30) {
        LOG(WARN, "Invalid address: %02X", addr);
        return -1;
    }

    chipSelect();
    uint8_t ret = _spi->write(addr);
    chipDeselect();

    // If debug is enabled, we wait for a brief interval, then send a NOP to get
    // the radio's status, then log it to the console
    if (_debugEnabled) {
        // wait a bit to allow the previous strobe to take effect
        // TODO: how long should this delay actually be?
        int delay = 2;
        Thread::wait(delay);

        chipSelect();
        uint8_t ret2 = _spi->write(CC1201_STROBE_SNOP);
        chipDeselect();

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
            strobe_names[addr - 0x30], rdy_n, CC1201_STATE_NAMES[state], delay,
            rdy2_n, CC1201_STATE_NAMES[state2]);
    }

    return ret;
}

uint8_t CC1201::mode() { return 0x1F & readReg(CC1201_MARCSTATE); }

void CC1201::reset() {
    idle();
    chipSelect();
    _spi->write(CC1201_STROBE_SRES);
    chipDeselect();

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

// TODO: make this method strobe into IDLE first?
uint8_t CC1201::freqUpdate() { return strobe(CC1201_STROBE_SAFC); }

float CC1201::freq() {
    // TODO: should freqUpdate() be called here?
    // freqUpdate();

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
