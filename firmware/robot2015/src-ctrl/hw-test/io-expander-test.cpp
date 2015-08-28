#include <mbed.h>
#include <rtos.h>

#include <vector>
#include <algorithm>

#include <helper-funcs.hpp>

#include "robot-devices.hpp"

DigitalOut good(LED1, 0);
DigitalOut bad1(LED2, 0);
DigitalOut bad2(LED3, 0);
DigitalOut pwr(LED4, 1);
Serial pc(RJ_SERIAL_RXTX);
I2C i2c(RJ_I2C_BUS);

bool testPass = false;
bool batchedResult = false;
std::vector<unsigned int> freq1;
std::vector<unsigned int> freq2;

int main()
{
    char cmd[2] = { 0x00 };

    pc.printf("START========= STARTING TEST =========\r\n\r\n");

    pwr = 0;
    RtosTimer live_light(imAlive, osTimerPeriodic, (void*)&pwr);
    live_light.start(250);

    pc.printf("--  Testing address space using 100kHz\r\n");
    // Test on low bus frequency
    i2c.frequency(100000);

    // For both frequencies, we check 1 additional address that should always fail for the
    // case where there's a response on every valid address of the IO expander.
    for (unsigned int addrOffset = 0; addrOffset < 0x09; addrOffset++) {
        bool failed = false;

        failed = i2c.read(0x40 | addrOffset, cmd, 1);

        if (!failed) {
            freq1.push_back(0x40 | addrOffset);
        }
    }

    pc.printf("--  Testing address space using 400kHz\r\n");
    // Test on high bus frequency
    i2c.frequency(400000);

    for (unsigned int addrOffset = 0; addrOffset < 0x09; addrOffset++) {
        bool failed = false;

        failed = i2c.read(0x40 | addrOffset, cmd, 1);

        if (!failed) {
            freq2.push_back(0x40 | addrOffset);
        }
    }

    // Test results
    pc.printf("\r\n100kHz Test:\t%s\r\n", freq1.empty() ? "FAIL" : "PASS");
    pc.printf("400kHz Test:\t%s\r\n", freq2.empty() ? "FAIL" : "PASS");

    // Merge the 2 vectors together & remove duplicate values
    freq1.insert(freq1.end(), freq2.begin(), freq2.end());
    sort( freq1.begin(), freq1.end() );
    freq1.erase( unique( freq1.begin(), freq1.end() ), freq1.end() );

    if ( freq1.size() > 9 ) {
        batchedResult = true;
    }

    // Final results of the test
    testPass = (freq1.size() == 1 || freq1.size() == 2 || freq1.size() == 8)  && (!batchedResult);

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
