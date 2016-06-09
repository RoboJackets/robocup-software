#include "fpga.hpp"

#include <algorithm>

#include <rtos.h>

#include "logger.hpp"
#include "software-spi.hpp"
#include "rj-macros.hpp"

FPGA* FPGA::Instance = nullptr;

namespace {
enum {
    CMD_EN_DIS_MTRS = 0x30,
    CMD_R_ENC_W_VEL = 0x80,
    CMD_READ_ENC = 0x91,
    CMD_READ_HALLS = 0x92,
    CMD_READ_DUTY = 0x93,
    CMD_READ_HASH1 = 0x94,
    CMD_READ_HASH2 = 0x95,
    CMD_CHECK_DRV = 0x96
};
}

FPGA::FPGA(std::shared_ptr<SharedSPI> sharedSPI, PinName nCs, PinName initB,
           PinName progB, PinName done)
    : SharedSPIDevice(sharedSPI, nCs, true),
      _initB(initB),
      _done(done),
      _progB(progB, PIN_OUTPUT, OpenDrain, 1) {
    setSPIFrequency(1000000);
}

bool FPGA::configure(const std::string& filepath) {
    // make sure the binary exists before doing anything
    FILE* fp = fopen(filepath.c_str(), "r");
    if (fp == nullptr) {
        LOG(FATAL, "No FPGA bitfile!");

        return false;
    }
    fclose(fp);

    // toggle PROG_B to clear out anything prior
    _progB = 0;
    Thread::wait(1);
    _progB = 1;

    // wait for the FPGA to tell us it's ready for the bitstream
    bool fpgaReady = false;
    for (int i = 0; i < 100; i++) {
        Thread::wait(10);

        // We're ready to start the configuration process when _initB goes high
        if (_initB == true) {
            fpgaReady = true;
            break;
        }
    }

    // show INIT_B error if it never went low
    if (!fpgaReady) {
        LOG(FATAL, "INIT_B pin timed out\t(PRE CONFIGURATION ERROR)");

        return false;
    }

    // Configure the FPGA with the bitstream file, this returns false if file
    // can't be opened
    if (send_config(filepath)) {
        // Wait some extra time in case the _done pin needs time to be asserted
        bool configSuccess = false;
        for (int i = 0; i < 1000; i++) {
            Thread::wait(1);
            if (_done == true) {
                configSuccess = !_initB;
                break;
            }
        }

        if (configSuccess) {
            // everything worked are we're good to go!
            _isInit = true;
            LOG(INF1, "DONE pin state:\t%s", _done ? "HIGH" : "LOW");

            return true;
        }

        LOG(FATAL, "DONE pin timed out\t(POST CONFIGURATION ERROR)");
    }

    LOG(FATAL, "FPGA bitstream write error");

    return false;
}

TODO(remove this hack once issue number 590 is closed)
#include "../../robot2015/src-ctrl/config/pins-ctrl-2015.hpp"

bool FPGA::send_config(const std::string& filepath) {
    const uint8_t bufSize = 50;

    // open the bitstream file
    FILE* fp = fopen(filepath.c_str(), "r");

    // send it out if successfully opened
    if (fp != nullptr) {
        size_t filesize;
        char buf[bufSize];

        // determine bitfile's size
        fseek(fp, 0, SEEK_END);
        filesize = ftell(fp);
        fseek(fp, 0, SEEK_SET);

        LOG(INF1, "Sending %s (%u bytes) out to the FPGA", filepath.c_str(),
            filesize);

        // select device & send bitfile
        chipSelect();
        for (size_t i = 0; i < filesize; i++) {
            bool breakOut = false;
            size_t readSize = fread(buf, 1, bufSize, fp);

            if (!readSize) break;

            for (size_t j = 0; j < bufSize; j++) {
                if (!_initB || _done) {
                    breakOut = true;
                    break;
                }

                _spi->write(buf[j]);
            }

            if (breakOut) break;
        }
        chipDeselect();

        fclose(fp);

        return true;
    }

    LOG(INIT, "FPGA configuration failed\r\n    Unable to open %s",
        filepath.c_str());

    return false;
}

uint8_t FPGA::read_halls(uint8_t* halls, size_t size) {
    uint8_t status;

    chipSelect();
    status = _spi->write(CMD_READ_HALLS);

    for (size_t i = 0; i < size; i++) halls[i] = _spi->write(0x00);

    chipDeselect();

    return status;
}

uint8_t FPGA::read_encs(uint16_t* enc_counts, size_t size) {
    uint8_t status;

    chipSelect();
    status = _spi->write(CMD_READ_ENC);

    for (size_t i = 0; i < size; i++) {
        enc_counts[i] = (_spi->write(0x00) << 8);
        enc_counts[i] |= _spi->write(0x00);
    }

    chipDeselect();

    return status;
}

uint8_t FPGA::read_duty_cycles(uint16_t* duty_cycles, size_t size) {
    uint8_t status;

    chipSelect();
    status = _spi->write(CMD_READ_DUTY);

    for (size_t i = 0; i < size; i++) {
        duty_cycles[i] = (_spi->write(0x00) << 8);
        duty_cycles[i] |= _spi->write(0x00);
    }

    chipDeselect();

    return status;
}

uint8_t FPGA::set_duty_cycles(uint16_t* duty_cycles, size_t size) {
    uint8_t status;

    // Check for valid duty cycles values
    for (size_t i = 0; i < size; i++)
        if (duty_cycles[i] > 0x3FF) return 0x7F;

    chipSelect();
    status = _spi->write(CMD_R_ENC_W_VEL);

    for (size_t i = 0; i < size; i++) {
        _spi->write(duty_cycles[i] & 0xFF);
        _spi->write(duty_cycles[i] >> 8);
    }

    chipDeselect();

    return status;
}

uint8_t FPGA::set_duty_get_enc(uint16_t* duty_cycles, size_t size_dut,
                               uint16_t* enc_deltas, size_t size_enc) {
    uint8_t status;

    // Check for valid duty cycles values
    for (size_t i = 0; i < size_dut; i++)
        if (duty_cycles[i] > 0x3FF) return 0x7F;

    chipSelect();
    status = _spi->write(CMD_R_ENC_W_VEL);

    for (size_t i = 0; i < size_enc; i++) {
        enc_deltas[i] = (_spi->write(duty_cycles[i] & 0xFF) << 8);
        enc_deltas[i] |= _spi->write(duty_cycles[i] >> 8);
    }

    chipDeselect();

    return status;
}

bool FPGA::git_hash(std::vector<uint8_t>& v) {
    bool dirty_bit;

    chipSelect();
    _spi->write(CMD_READ_HASH1);

    for (size_t i = 0; i < 10; i++) v.push_back(_spi->write(0x00));

    chipDeselect();
    chipSelect();

    _spi->write(CMD_READ_HASH2);

    for (size_t i = 0; i < 11; i++) v.push_back(_spi->write(0x00));

    chipDeselect();

    // store the dirty bit for returning
    dirty_bit = (v.back() & 0x01);
    // remove the last byte
    v.pop_back();

    // reverse the bytes
    std::reverse(v.begin(), v.end());

    return dirty_bit;
}

void FPGA::gate_drivers(std::vector<uint16_t>& v) {
    chipSelect();

    _spi->write(CMD_CHECK_DRV);

    // each halfword is structured as follows (MSB -> LSB):
    // | nibble 2: | GVDD_OV  | FAULT    | GVDD_UV  | PVDD_UV  |
    // | nibble 1: | OTSD     | OTW      | FETHA_OC | FETLA_OC |
    // | nibble 0: | FETHB_OC | FETLB_OC | FETHC_OC | FETLC_OC |
    for (size_t i = 0; i < 10; i++) {
        uint16_t tmp = _spi->write(0x00);
        tmp |= (_spi->write(0x00) << 8);
        v.push_back(tmp);
    }

    chipDeselect();
}

uint8_t FPGA::motors_en(bool state) {
    uint8_t status;

    chipSelect();
    status = _spi->write(CMD_EN_DIS_MTRS | (state << 7));
    chipDeselect();

    return status;
}

uint8_t FPGA::watchdog_reset() {
    motors_en(false);
    return motors_en(true);
}

bool FPGA::isReady() { return _isInit; }
