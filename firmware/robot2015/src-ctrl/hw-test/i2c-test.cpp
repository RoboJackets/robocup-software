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

    for (unsigned int i = 0; i < 0xFF; i++) {
        bool response = false;

        response = i2c.read(i, cmd, 1);
        response |= i2c.read(i, cmd, 2);

        if (!response) {
            freq1.push_back(i);
        }
    }

    pc.printf("--  Testing address space using 400kHz\r\n");
    // Test on high bus frequency
    i2c.frequency(400000);

    for (unsigned int i = 0; i < 0xFF; i++) {
        bool response = false;

        response = i2c.read(i, cmd, 1);
        response |= i2c.read(i, cmd, 2);

        if (!response) {
            freq2.push_back(i);
        }
    }

    // Test results
    pc.printf("\r\n100kHz Test:\t%s\r\n", freq1.empty() ? "FAIL" : "PASS");
    pc.printf("400kHz Test:\t%s\r\n", freq2.empty() ? "FAIL" : "PASS");

    // Merge the 2 vectors together & remove duplicate values
    freq1.insert(freq1.end(), freq2.begin(), freq2.end());
    sort( freq1.begin(), freq1.end() );
    freq1.erase( unique( freq1.begin(), freq1.end() ), freq1.end() );

    pc.printf("ADDRS PASS:\t%u\r\nADDRS FAIL:\t%u\r\n", freq1.size(), 0xFF - freq1.size());

    if ( freq1.size() > 0xF0 ) {
        batchedResult = true;
    }

    for (std::vector<unsigned int>::iterator it = freq1.begin() ; it != freq1.end(); ++it)
        pc.printf("    0x%02X\r\n", *it);

    // Final results of the test
    testPass = (!freq1.empty()) && (!batchedResult);

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
