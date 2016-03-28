#include "fpga.hpp"

#include <rtos.h>
#include <logger.hpp>
#include <software-spi.hpp>

#include "commands.hpp"

FPGA* FPGA::instance = nullptr;
bool FPGA::isInit = false;

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
    : spi(sharedSPI),
      nCs(nCs),
      initB(initB),
      progB(progB, PIN_OUTPUT, OpenDrain, 1),
      done(done) {
    this->nCs = 1;  // start out de-asserted
}

FPGA* FPGA::Initialize(shared_ptr<SharedSPI> sharedSPI) {
    instance = new FPGA(sharedSPI, RJ_FPGA_nCS, RJ_FPGA_INIT_B, RJ_FPGA_PROG_B,
                        RJ_FPGA_DONE);

    return instance;
}

FPGA* FPGA::Instance() { return instance; }

bool FPGA::configure(const std::string& filepath) {
    // make sure the binary exists before doing anything
    FILE* fp = fopen(filepath.c_str(), "r");
    if (fp == nullptr) {
        LOG(FATAL, "No FPGA bitfile!");

        return false;
    }
    fclose(fp);

    // toggle PROG_B to clear out anything prior
    progB = 0;
    Thread::wait(1);
    progB = 1;
    Thread::wait(1);

    // wait for the FPGA to tell us it's ready for the bitstream
    bool fpgaReady = false;
    for (int i = 0; i < 100; i++) {
        Thread::wait(10);

        // We're ready to start the configuration process when initB goes high
        if (initB == true) {
            fpgaReady = true;
            break;
        }
    }

    // show INIT_B error if it never went low
    if (fpgaReady) {
        LOG(FATAL, "INIT_B pin timed out\t(PRE CONFIGURATION ERROR)");

        return false;
    }

    // Configure the FPGA with the bitstream file
    if (send_config(filepath)) {
        LOG(FATAL, "FPGA bitstream write error");

        return false;
    } else {
        // Wait some extra time in case the done pin needs time to be asserted
        bool configureDone = false;
        for (int i = 0; i < 1000; i++) {
            Thread::wait(1);
            if (done == true) {
                configureDone = true;
                break;
            }
        }

        if (!configureDone) {
            LOG(FATAL, "DONE pin timed out\t(POST CONFIGURATION ERROR)");
            return false;
        } else {
            // everything worked are we're good to go!
            LOG(INF1, "DONE pin state:\t%s", done ? "HIGH" : "LOW");

            isInit = true;

            // spi->frequency(1000000); // TODO(justin): remove?

            return true;
        }
    }
}

bool FPGA::send_config(const std::string& filepath) {
    char buf[10];

    // open the bitstream file
    FILE* fp = fopen(filepath.c_str(), "r");

    // send it out if successfully opened
    if (fp != nullptr) {
        // MISO & MOSI are intentionally switched here
        // defaults to 8 bit field size with CPOL = 0 & CPHA = 0
        SoftwareSPI spi(RJ_SPI_MISO, RJ_SPI_MOSI, RJ_SPI_SCK);

        size_t read_byte;

        fseek(fp, 0, SEEK_END);
        size_t filesize = ftell(fp);
        fseek(fp, 0, SEEK_SET);

        LOG(INF1, "Sending %s (%u bytes) out to the FPGA", filepath.c_str(),
            filesize);

        fpga_select();

        do {
            read_byte = fread(buf, 1, 1, fp);

            if (read_byte == 0) break;

            spi.write(buf[0]);

        } while (initB || !done);

        fpga_deselect();

        fclose(fp);

        return false;

    } else {
        LOG(INIT, "FPGA configuration failed\r\n    Unable to open %s",
            filepath.c_str());

        return true;
    }
}

uint8_t FPGA::read_halls(uint8_t* halls, size_t size) {
    uint8_t status;

    fpga_select();
    status = spi->write(CMD_READ_HALLS);

    for (size_t i = 0; i < size; i++) halls[i] = spi->write(0x00);

    fpga_deselect();

    return status;
}

uint8_t FPGA::read_encs(uint16_t* enc_counts, size_t size) {
    uint8_t status;

    fpga_select();
    status = spi->write(CMD_READ_ENC);

    for (size_t i = 0; i < size; i++) {
        enc_counts[i] = (spi->write(0x00) << 8);
        enc_counts[i] |= spi->write(0x00);
    }

    fpga_deselect();

    return status;
}

uint8_t FPGA::read_duty_cycles(uint16_t* duty_cycles, size_t size) {
    uint8_t status;

    fpga_select();
    status = spi->write(CMD_READ_DUTY);

    for (size_t i = 0; i < size; i++) {
        duty_cycles[i] = (spi->write(0x00) << 8);
        duty_cycles[i] |= spi->write(0x00);
    }

    fpga_deselect();

    return status;
}

uint8_t FPGA::set_duty_cycles(uint16_t* duty_cycles, size_t size) {
    uint8_t status;

    // Check for valid duty cycles values
    for (size_t i = 0; i < size; i++)
        if (duty_cycles[i] > 0x3FF) return 0x7F;

    fpga_select();
    status = spi->write(CMD_R_ENC_W_VEL);

    for (size_t i = 0; i < size; i++) {
        spi->write(duty_cycles[i] & 0xFF);
        spi->write(duty_cycles[i] >> 8);
    }

    fpga_deselect();

    return status;
}

uint8_t FPGA::set_duty_get_enc(uint16_t* duty_cycles, size_t size_dut,
                               uint16_t* enc_deltas, size_t size_enc) {
    uint8_t status;

    // Check for valid duty cycles values
    for (size_t i = 0; i < size_dut; i++)
        if (duty_cycles[i] > 0x3FF) return 0x7F;

    fpga_select();
    status = spi->write(CMD_R_ENC_W_VEL);

    for (size_t i = 0; i < size_enc; i++) {
        enc_deltas[i] = (spi->write(duty_cycles[i] & 0xFF) << 8);
        enc_deltas[i] |= spi->write(duty_cycles[i] >> 8);
    }

    fpga_deselect();

    return status;
}

bool FPGA::git_hash(std::vector<uint8_t>& v) {
    bool dirty_bit;

    fpga_select();
    spi->write(CMD_READ_HASH1);

    for (size_t i = 0; i < 10; i++) v.push_back(spi->write(0x00));

    fpga_deselect();
    fpga_select();

    spi->write(CMD_READ_HASH2);

    for (size_t i = 0; i < 11; i++) v.push_back(spi->write(0x00));

    fpga_deselect();

    // store the dirty bit for returning
    dirty_bit = (v.back() & 0x01);
    // remove the last byte
    v.pop_back();

    // reverse the bytes
    std::reverse(v.begin(), v.end());

    return dirty_bit;
}

void FPGA::gate_drivers(std::vector<uint16_t>& v) {
    fpga_select();

    spi->write(CMD_CHECK_DRV);

    // each halfword is structured as follows:
    // GVDD_OV | FAULT | GVDD_UV | PVDD_UV | OTSD | OTW | FETHA_OC | FETLA_OC |
    // FETHB_OC | FETLB_OC | FETHC_OC | FETLC_OC
    for (size_t i = 0; i < 10; i++) {
        uint16_t tmp = spi->write(0x00);
        tmp |= (spi->write(0x00) << 8);
        v.push_back(tmp);
    }

    fpga_deselect();
}

uint8_t FPGA::motors_en(bool state) {
    uint8_t status;

    fpga_select();
    status = spi->write(CMD_EN_DIS_MTRS | (state << 7));
    fpga_deselect();

    return status;
}

uint8_t FPGA::watchdog_reset() {
    motors_en(false);
    return motors_en(true);
}

bool FPGA::isReady() { return isInit; }

void FPGA::fpga_select() {
    spi->lock();
    nCs = 0;
}

void FPGA::fpga_deselect() {
    nCs = 1;
    spi->lock();
}
