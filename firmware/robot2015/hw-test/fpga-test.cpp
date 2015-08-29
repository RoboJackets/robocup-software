#include <mbed.h>
#include <rtos.h>

#include <string>

#include <helper-funcs.hpp>
#include <SWSPI.h>

#include "robot-devices.hpp"

const std::string filename = "rj-fpga.nib";
const std::string filesystemname = "local";
const std::string filepath = "/" + filesystemname + "/" + filename;

DigitalOut good(LED1, 0);
DigitalOut bad1(LED2, 0);
DigitalOut bad2(LED3, 0);
DigitalOut pwr(LED4, 1);
Serial pc(RJ_SERIAL_RXTX);

DigitalOut trigger(RJ_SPARE_IO, 0);     // line for triggering a logic analyzer for debugging
DigitalOut radio_nCS(RJ_RADIO_nCS, 1);  // keep the radio chip select line unselected
DigitalOut cs(RJ_FPGA_nCS, 1);          // don't really care. no used for configuration
DigitalIn done(RJ_FPGA_DONE);
//DigitalInOut prog_b(RJ_FPGA_PROG_B, PIN_OUTPUT, PullUp, 1);
DigitalOut prog_b(RJ_FPGA_PROG_B, 1);
DigitalInOut init_b(RJ_FPGA_INIT_B, PIN_INPUT, PullUp, 1);

bool testPass = false;
bool batchedResult = false;

void start_start_config(DigitalOut& p)
{
    p = !p;
    Thread::wait(1);
    p = !p;
    Thread::wait(1);
    p = 0;
}

bool fpgaInit(void)
{
    trigger = !trigger;

    LocalFileSystem local(filesystemname.c_str());
    // SPI spi(RJ_SPI_BUS);
    SWSPI spi(RJ_SPI_MISO, NC, RJ_SPI_SCK);
    char buf[10];

    //  8 bits per write, mode 3
    spi.format(8, 0);
    spi.frequency(RJ_FPGA_SPI_FREQ);

    FILE* fp = fopen(filepath.c_str(), "r");

    if ( fp != nullptr ) {
        fseek (fp, 0, SEEK_END);
        size_t filesize = ftell(fp);
        size_t divisor = 15;
        int mod_by = filesize / divisor;
        size_t count;

        pc.printf("--  successfully opened %s (%u bytes)\r\n", filename.c_str(), filesize);

        do {
            size_t count = fread(buf, 1, 1, fp);

            if (count == 0) {
                pc.printf("--  100%%\r\n");
                break;
            }

            // reading any response is usesless because the fpga
            // config's pin is unconnected for data out & it always
            // returns high (if it was connected)
            spi.write(buf[0]);

            // count++;
            if (count % mod_by == 0) {
                pc.printf(
                    "--  %3.0f%%\t(0x%02X)\r\n",
                    static_cast<float>((count * 100) / filesize)
                );
            }
        } while (init_b.read() && !(done.read()));

        fclose(fp);

        pc.printf("--  closed %s\r\n", filename.c_str());
        pc.printf("--  wrote %u bytes\r\n", count);

        trigger = !trigger;

        return count != filesize;
    } else {

        batchedResult = true;
        pc.printf("--  could not open %s", filepath.c_str());

        return 1;
    }
}

int main()
{
    trigger = !trigger;
    Thread::wait(2);
    trigger = !trigger;

    int j = 0;

    pc.printf("START========= STARTING TEST =========\r\n\r\n");

    pwr = 0;
    RtosTimer live_light(imAlive, osTimerPeriodic, (void*)&pwr);
    live_light.start(250);

    // strobe the prob_b pin to reset any previous configurations for the FPGA
    start_start_config(prog_b);

    // wait for the FPGA to tell us it's ready for the bitstream
    for (int i = 0; i < 10; i++) {
        j++;
        Thread::wait(100);

        // We're ready to start the configuration process when init_b goes high
        if (init_b.read() == true)
            break;
    }

    // show init_b error if it never went low
    if (j == 10) {
        pc.printf("--  INIT_B pin timed out\t(PRE CONFIGURATION ERROR)\r\n");
        batchedResult = true;
    }

    // Configure the FPGA with the bitstream file
    bool configFail = fpgaInit();

    // Wait some extra time in case the done pin needs time to be asserted
    j = 0;

    for (int i = 0; i < 1000; i++) {
        j++;
        Thread::wait(1);

        if (done.read() == true)
            break;
    }

    if (j == 1000 && configFail == false) {
        pc.printf("--  done pin timed out\t(POST CONFIGURATION ERROR)\r\n", configFail);
        batchedResult = true;
    } else if (j == 1000 && configFail == true) {
        pc.printf("--  done pin timed out\t(CONFIGURATION WRITE ERROR)\r\n", configFail);
        batchedResult = true;
    }

    // Final results of the test
    testPass = done && !batchedResult;

    pc.printf("\r\n=================================\r\n");
    pc.printf("========== TEST %s ==========\r\n", testPass ? "PASSED" : "FAILED");
    pc.printf("=================================DONE");

    // Turn on the corresponding LED(s)
    good = testPass;
    bad1 = !testPass;
    bad2 = !testPass;
    live_light.stop();
    pwr = testPass;

    while (true) {
        // Never return
        wait(2.0);
    }
}
