#include "fpga.hpp"

#include <rtos.h>
#include <logger.hpp>
#include <software-spi.hpp>

#include "commands.hpp"


bool FPGA::isInit = false;

namespace
{
enum {
    CMD_EN_DIS_MTRS = 0x30,
    CMD_R_ENC_W_VEL = 0x80,
    CMD_READ_HALLS  = 0x92,
    CMD_READ_DUTY   = 0x93,
    CMD_READ_HASH   = 0x94
};
}


FPGA::FPGA(PinName _mosi, PinName _miso, PinName _sck, PinName _cs, PinName _progB, PinName _initB, PinName _done)
    : spi(_mosi, _miso, _sck),
      cs(_cs, 1),
      progB(_progB, PIN_OUTPUT, OpenDrain, 1),
      initB(_initB),
      done(_done)
{}


FPGA::~FPGA(void)
{
    isInit = false;
}


/**
 * [FPGA::Init Setup the FPGA interface]
 * @return  [The initialization error code.]
 */
bool FPGA::Init(const std::string& filepath)
{
    int j = 0;

    // toggle PROG_B to clear out anything prior
    progB = !progB;
    Thread::wait(1);
    progB = !progB;
    Thread::wait(1);

    // wait for the FPGA to tell us it's ready for the bitstream
    for (int i = 0; i < 100; i++) {
        j++;
        Thread::wait(10);

        // We're ready to start the configuration process when initB goes high
        if (initB == true)
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

            if (done == true) break;
        }

        if (j == 1000) {
            LOG(FATAL, "DONE pin timed out\t(POST CONFIGURATION ERROR)");

            return false;
        }
        // everything worked are we're good to go!
        else {
            LOG(INF1, "DONE pin state:\t%s", done ? "HIGH" : "LOW");

            isInit = true;

            cs = 1;

            spi = SPI(RJ_SPI_BUS);
            spi.format(8, 0);
            spi.frequency(2000000);

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

        } while (initB == true || done == false);

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

    cs = !cs;
    status = spi.write(CMD_READ_HALLS);

    for (int i = 0; i < 5; i++)
        halls[i] = spi.write(0x00);

    cs = !cs;

    return status;
}


uint8_t FPGA::read_duty_cycles(uint16_t* duty_cycles, size_t size)
{
    uint8_t status;

    cs = !cs;
    status = spi.write(CMD_READ_DUTY);

    for (int i = 0; i < 5; i++) {
        duty_cycles[i]  = (spi.write(0x00) << 8);
        duty_cycles[i] |= spi.write(0x00);
    }

    cs = !cs;

    return status;
}


uint8_t FPGA::set_duty_get_enc(uint16_t* duty_cycles, size_t size_dut, uint16_t* enc_deltas, size_t size_enc)
{
    uint8_t status;

    // Check for valid duty cycles values
    for (int i = 0; i < 5; i++)
        if (duty_cycles[i] > 0x3FF) return 0x7F;

    cs = !cs;
    status = spi.write(CMD_R_ENC_W_VEL);

    for (int i = 0; i < 5; i++) {
        enc_deltas[i]  = (spi.write(duty_cycles[i] & 0xFF) << 8);
        enc_deltas[i] |= spi.write(duty_cycles[i] >> 8);
    }

    cs = !cs;

    return status;
}


uint32_t FPGA::git_hash(void)
{
    uint32_t hash = 0;

    cs = !cs;
    spi.write(CMD_READ_HASH);

    for (int i = 0; i < 4; i++)
        hash |= (spi.write(0x00) << (8 * i));

    cs = !cs;
}

uint8_t FPGA::motors_en(bool state)
{
    uint8_t status;

    cs = !cs;
    status = spi.write(CMD_EN_DIS_MTRS | (state << 7));
    cs = !cs;

    return status;
}
