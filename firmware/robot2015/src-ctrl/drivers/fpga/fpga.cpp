#include "fpga.hpp"

#include <rtos.h>
#include <logger.hpp>
#include <software-spi.hpp>

#include "commands.hpp"

FPGA* FPGA::instance = nullptr;

bool FPGA::isInit = false;

namespace
{
enum {
    CMD_EN_DIS_MTRS = 0x30,
    CMD_R_ENC_W_VEL = 0x80,
    CMD_READ_ENC    = 0x91,
    CMD_READ_HALLS  = 0x92,
    CMD_READ_DUTY   = 0x93,
    CMD_READ_HASH   = 0x94
};
}

FPGA* FPGA::Instance()
{
    if ( instance == nullptr ) {
        instance = new FPGA;
        instance->spi = new SPI(RJ_SPI_BUS);
        instance->progB = new DigitalInOut(RJ_FPGA_PROG_B, PIN_OUTPUT, OpenDrain, 1);
        instance->initB = new DigitalIn(RJ_FPGA_INIT_B);
        instance->done = new DigitalIn(RJ_FPGA_DONE);
        instance->cs = new DigitalOut(RJ_FPGA_nCS);
    }

    return instance;
}

/**
 * [FPGA::Init Setup the FPGA interface]
 * @return  [The initialization error code.]
 */
bool FPGA::Init(const std::string& filepath)
{
    int j = 0;

    // toggle PROG_B to clear out anything prior
    *progB = !(*progB);
    Thread::wait(1);
    *progB = !(*progB);
    Thread::wait(1);

    // wait for the FPGA to tell us it's ready for the bitstream
    for (int i = 0; i < 100; i++) {
        j++;
        Thread::wait(10);

        // We're ready to start the configuration process when initB goes high
        if (*initB == true)
            break;
    }

    // show INIT_B error if it never went low
    if (j == 10) {
        LOG(FATAL, "INIT_B pin timed out\t(PRE CONFIGURATION ERROR)");
        return false;
    }


    // Configure the FPGA with the bitstream file
    if (send_config(filepath)) {
        LOG(FATAL, "FPGA bitstream write error");

        return false;
    }

    else {
        // Wait some extra time in case the done pin needs time to be asserted
        j = 0;

        for (int i = 0; i < 1000; i++) {
            Thread::wait(1); j++;

            if (*done == true) break;
        }

        if (j == 1000) {
            LOG(FATAL, "DONE pin timed out\t(POST CONFIGURATION ERROR)");

            return false;
        }
        // everything worked are we're good to go!
        else {
            LOG(INF1, "DONE pin state:\t%s", *done ? "HIGH" : "LOW");

            isInit = true;

            cs->write(1);
            spi->format(8, 0);
            spi->frequency(1000000);

            return true;
        }
    }
}


bool FPGA::send_config(const std::string& filepath)
{
    char buf[10];

    // open the bitstream file
    FILE* fp = fopen(filepath.c_str(), "r");

    // send it out if successfully opened
    if ( fp != nullptr ) {
        // MISO & MOSI are intentionally switched here
        // defaults to 8 bit field size with CPOL = 0 & CPHA = 0
        SoftwareSPI spi(RJ_SPI_MISO, RJ_SPI_MOSI, RJ_SPI_SCK);

        fseek (fp, 0, SEEK_END);
        size_t filesize = ftell(fp);
        fseek (fp, 0, SEEK_SET);

        LOG(INF1, "Sending %s (%u bytes) out to the FPGA", filepath.c_str(), filesize);

        size_t read_byte;

        do {
            read_byte = fread(buf, 1, 1, fp);

            if (read_byte == 0) break;

            spi.write(buf[0]);

        } while (*initB == true || *done == false);

        fclose(fp);

        return false;
    } else {

        LOG(INIT, "FPGA configuration failed\r\n    Unable to open %s", filepath.c_str());

        return true;
    }
}


uint8_t FPGA::read_halls(uint8_t* halls, size_t size)
{
    uint8_t status;

    mutex.lock();
    *cs = !(*cs);
    status = spi->write(CMD_READ_HALLS);

    for (int i = 0; i < size; i++)
        halls[i] = spi->write(0x00);

    *cs = !(*cs);
    mutex.unlock();

    return status;
}

uint8_t FPGA::read_encs(uint16_t* enc_counts, size_t size)
{
    uint8_t status;

    mutex.lock();
    *cs = !(*cs);
    status = spi->write(CMD_READ_ENC);

    for (int i = 0; i < size; i++) {
        enc_counts[i]  = (spi->write(0x00) << 8);
        enc_counts[i] |= spi->write(0x00);
    }

    *cs = !(*cs);
    mutex.unlock();

    return status;
}


uint8_t FPGA::read_duty_cycles(uint16_t* duty_cycles, size_t size)
{
    uint8_t status;

    mutex.lock();
    *cs = !(*cs);
    status = spi->write(CMD_READ_DUTY);

    for (int i = 0; i < size; i++) {
        duty_cycles[i]  = (spi->write(0x00) << 8);
        duty_cycles[i] |= spi->write(0x00);
    }

    *cs = !(*cs);
    mutex.unlock();

    return status;
}


uint8_t FPGA::set_duty_get_enc(uint16_t* duty_cycles, size_t size_dut, uint16_t* enc_deltas, size_t size_enc)
{
    uint8_t status;

    // Check for valid duty cycles values
    for (int i = 0; i < size_dut; i++)
        if (duty_cycles[i] > 0x3FF) return 0x7F;

    mutex.lock();
    *cs = !(*cs);
    status = spi->write(CMD_R_ENC_W_VEL);

    for (int i = 0; i < size_enc; i++) {
        enc_deltas[i]  = (spi->write(duty_cycles[i] & 0xFF) << 8);
        enc_deltas[i] |= spi->write(duty_cycles[i] >> 8);
    }

    *cs = !(*cs);
    mutex.unlock();

    return status;
}


uint32_t FPGA::git_hash()
{
    return 0xFF;

    // uint32_t hash = 0;

    // mutex.lock();
    // *cs = !(*cs);
    // spi->write(CMD_READ_HASH);

    // for (int i = 0; i < 4; i++)
    //     hash |= (spi->write(0x00) << (8 * i));

    // *cs = !(*cs);
    // mutex.unlock();

    // return hash;
}

uint8_t FPGA::motors_en(bool state)
{
    uint8_t status;

    mutex.lock();
    *cs = !(*cs);
    status = spi->write(CMD_EN_DIS_MTRS | (state << 7));
    *cs = !(*cs);
    mutex.unlock();

    return status;
}
