#include "CC1201.hpp"

#include "logger.hpp"
#include "assert.hpp"

CC1201::CC1201(PinName mosi, PinName miso, PinName sck, PinName cs,
               PinName intPin, const registerSetting_t* regs, size_t len, int rssiOffset)
    : CommLink(mosi, miso, sck, cs, intPin) {
    // set initial configuration
    setConfig(regs, len);

    _offset_reg_written = false;
    reset();
    set_rssi_offset(rssiOffset);
    selfTest();

    if (_isInit == true) {
        LOG(INIT, "CC1201 ready!");
        CommLink::ready();
    }
}

int32_t CC1201::sendData(uint8_t* buf, uint8_t size) {
    // Return if there's no functional radio transceiver - the system will
    // lockup otherwise
    if (_isInit == false) return -1;

    if (size != (buf[0] + 1)) {
        LOG(SEVERE,
            "Packet size is inconsistent with expected number of "
            "bytes! %u bytes expected, %u bytes found in buffer.",
            size, buf[0]);
        return COMM_FUNC_BUF_ERR;
    }

    strobe(CC1201_STROBE_SFTX);

    // Send the data to the CC1201.
    uint8_t device_state = writeReg(CC1201_SINGLE_TXFIFO, buf, size);

    // Enter the TX state.
    if ((device_state & CC1201_STATE_TXFIFO_ERROR) == CC1201_STATE_TXFIFO_ERROR) {
        LOG(WARN, "STATE AT TX ERROR: 0x%02X", device_state);
        flush_tx();  // flush the TX buffer & return if the FIFO is in a corrupt
        // state

        // set in IDLE mode and strobe back into RX to ensure the states will
        // fall through calibration then return
        idle();
        strobe(CC1201_STROBE_SRX);

        return COMM_DEV_BUF_ERR;
    } else {
        strobe(CC1201_STROBE_STX);  // Enter TX mode
    }

    // Wait until radio's TX buffer is emptied
    uint8_t bts = 1;

    do {
        bts = readReg(CC1201_NUM_TXBYTES);
        Thread::wait(2);

    } while (bts != 0);

    return COMM_SUCCESS;
}

int32_t CC1201::getData(uint8_t* buf, uint8_t* len) {
    uint8_t device_state = freqUpdate();  // update frequency offset estimate &
    // get the current state while at it
    uint8_t num_rx_bytes = readReg(CC1201_NUM_RXBYTES);

    if (((*len) + 2) < num_rx_bytes) {
        LOG(SEVERE, "%u bytes in RX FIFO with given buffer size of %u bytes.",
            num_rx_bytes, *len);

        return COMM_FUNC_BUF_ERR;
    }

    if ((device_state & CC1201_STATE_RXFIFO_ERROR) == CC1201_STATE_RXFIFO_ERROR) {
        flush_rx();  // flush RX FIFO buffer and place back into RX state
        strobe(CC1201_STROBE_SRX);

        return COMM_DEV_BUF_ERR;
    }

    if (num_rx_bytes > 0) {
        device_state = readReg(CC1201_SINGLE_RXFIFO, buf, num_rx_bytes);
        *len = num_rx_bytes;

        LOG(INF3, "Bytes in RX buffer: %u\r\nPayload bytes: %u", num_rx_bytes,
            buf[0]);
    } else {
        return COMM_NO_DATA;
    }

    update_rssi();
    // return back to RX mode
    strobe(CC1201_STROBE_SRX);

    return COMM_SUCCESS;
}

/**
 * reads a standard register
 */
uint8_t CC1201::readReg(uint16_t addr) {
    uint8_t returnVal;

    toggle_cs();
    if (addr >= CC1201_EXTENDED_ACCESS) {
        _spi->write(CC1201_EXTENDED_ACCESS | CC1201_READ);
        _spi->write(addr & 0xFF);
    } else {
        _spi->write(addr | CC1201_READ);
    }
    returnVal = _spi->write(0x00);
    toggle_cs();

    return returnVal;
}
uint8_t CC1201::readReg(uint16_t addr, uint8_t* buffer, uint8_t len) {
    uint8_t status_byte;

    toggle_cs();
    if (addr >= CC1201_EXTENDED_ACCESS) {
        status_byte = _spi->write(CC1201_EXTENDED_ACCESS | CC1201_READ | CC1201_BURST);
        _spi->write(addr & 0xFF);
    } else {
        status_byte = _spi->write(addr | CC1201_READ | CC1201_BURST);
    }
    for (uint8_t i = 0; i < len; i++) buffer[i] = _spi->write(0x00);
    toggle_cs();

    return status_byte;
}

uint8_t CC1201::writeReg(uint16_t addr, uint8_t value) {
    uint8_t status_byte;

    toggle_cs();
    if (addr >= CC1201_EXTENDED_ACCESS) {
        status_byte = _spi->write(CC1201_EXTENDED_ACCESS);
        _spi->write(addr & 0xFF);
    } else {
        status_byte = _spi->write(addr);
    }
    _spi->write(value);
    toggle_cs();

    return status_byte;
}

uint8_t CC1201::writeReg(uint16_t addr, const uint8_t* buffer, uint8_t len) {
    uint8_t status_byte;

    toggle_cs();
    if (addr >= CC1201_EXTENDED_ACCESS) {
        status_byte = _spi->write(CC1201_EXTENDED_ACCESS);
        _spi->write(addr & 0xFF | CC1201_BURST); // write lower byte of address
    } else {
        status_byte = _spi->write(addr | CC1201_BURST); // write lower byte of address
    }
    for (uint8_t i = 0; i < len; i++) _spi->write(buffer[i]);
    toggle_cs();

    return status_byte;
}

uint8_t CC1201::strobe(uint8_t addr) {
    if (addr > 0x3d || addr < 0x30) {
        LOG(WARN, "Invalid address: %02X", addr);
        return -1;
    }

    toggle_cs();
    uint8_t ret = _spi->write(addr);
    toggle_cs();

    return ret;
}

uint8_t CC1201::mode() {
    return 0x1F & readReg(CC1201_MARCSTATE);
}

uint8_t CC1201::status(uint8_t addr) { return strobe(addr); }

void CC1201::reset() {
    idle();
    toggle_cs();
    _spi->write(CC1201_STROBE_SRES);
    toggle_cs();

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
    if (_isInit == true) return 0;

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

bool CC1201::isConnected() { return _isInit; }

void CC1201::flush_tx() {
    idle();
    strobe(CC1201_STROBE_SFTX);
    LOG(WARN, "%u bytes flushed from TX FIFO buffer.",
        readReg(CC1201_NUM_TXBYTES));
}

void CC1201::flush_rx() {
    idle();
    strobe(CC1201_STROBE_SFRX);
    LOG(WARN, "%u bytes flushed from RX FIFO buffer.",
        readReg(CC1201_NUM_RXBYTES));
}

void CC1201::calibrate() {
    idle();
    strobe(CC1201_STROBE_SCAL);
}

void CC1201::update_rssi() {
    uint8_t offset = 0;

    // Only use the top MSB for simplicity. 1 dBm resolution.
    if (_offset_reg_written) {
        offset = readReg(CC1201_RSSI1);
        _rssi = static_cast<float>((int8_t)twos_compliment(offset));

        LOG(INF3, "RSSI is from device.");
    } else {
        _rssi = 0.0;
    }

    LOG(INF3, "RSSI Register Val: 0x%02X", offset);
}

float CC1201::rssi() { return _rssi; }

uint8_t CC1201::idle() {
    uint8_t status_byte = strobe(CC1201_STROBE_SIDLE);

    if (_isInit == false) return status_byte;

    // block until we've reached idle
    while (mode() != 0x01)
        ;

    return status_byte;
}

uint8_t CC1201::rand() {
    writeReg(CC1201_RNDGEN, 0x80);
    return readReg(CC1201_RNDGEN);
}

uint8_t CC1201::freqUpdate() { return strobe(CC1201_STROBE_SAFC); }

float CC1201::freq() {
    uint8_t buf[5];
    uint16_t freq_offset;
    uint32_t freq_base;
    float freq;

    freqUpdate();

    readReg(CC1201_FREQOFF1, buf, 5);

    freq_offset = (buf[0] << 8) | (buf[1]);
    freq_offset = (~freq_offset) + 1;
    freq_base = (buf[2] << 16) | (buf[3] << 8) | (buf[4]);

    freq = 40 * static_cast<float>((freq_base >> 16) + (freq_offset >> 18));
    freq /= 4;

    LOG(INF2, "Operating Frequency: %3.2f MHz", freq);

    return freq;
}

bool CC1201::isLocked() {
    // This is only valid in RX, TX, & FSTXON
    return readReg(CC1201_FSCAL_CTRL) & 0x01;
}

void CC1201::set_rssi_offset(int8_t offset) {
    // HAVING THIS MEANS OFFSET MUST ONLY BE SET ONCE FOR THE CLASS
    if (_offset_reg_written == true) return;

    _offset_reg_written = true;
    writeReg(CC1201_AGC_GAIN_ADJUST, twos_compliment(offset));
}

void CC1201::setConfig(const registerSetting_t* regs, size_t len) {
    if (!regs) return;

    for (size_t i = 0; i < len; ++i) {
        writeReg(regs[i].addr, regs[i].value);
    }
}
