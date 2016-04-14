#include "KickerBoard.hpp"
#include <tuple>

using namespace std;

KickerBoard::KickerBoard(PinName mosi, PinName miso, PinName sck,
                         PinName nReset, PinName n_cs, const string& progFilename)
    : AVR910(mosi, miso, sck, nReset), _filename(progFilename),
    spi(mosi, miso, sck), n_kick_select(n_cs) {
        spi.format(8, 0);
        spi.frequency(8000);
    }

bool KickerBoard::verify_param(const char* name, char expected,
                               int (AVR910::*paramMethod)(), char mask,
                               bool verbose) {
    if (verbose) printf("Checking %s...", name);
    int val = (*this.*paramMethod)();
    bool success = ((val & mask) == expected);
    if (verbose) {
        if (success)
            printf("done\r\n");
        else
            printf("Got unexpected value: 0x%X\r\n", val);
    }

    return success;
}

bool KickerBoard::flash(bool onlyIfDifferent, bool verbose) {
    // Check a few parameters before attempting to flash to ensure that we have
    // the right chip and it's connected correctly.
    auto checks = {
        make_tuple("Vendor ID", ATMEL_VENDOR_CODE, &AVR910::readVendorCode,
                   0xFF),
        make_tuple("Part Family", AVR_FAMILY_ID,
                   &AVR910::readPartFamilyAndFlashSize, AVR_FAMILY_MASK),
        make_tuple("Device ID", ATTINY84A_DEVICE_ID, &AVR910::readPartNumber,
                   0xFF),
    };
    for (auto& check : checks) {
        if (!verify_param(get<0>(check), get<1>(check), get<2>(check),
                          get<3>(check), verbose)) {
            return false;
        }
    }

    //  Open binary file to write to AVR.
    if (verbose) printf("Opening kicker firmware file...");
    FILE* fp = fopen(_filename.c_str(), "r");

    if (fp == NULL) {
        if (verbose)
            printf(
                "failed\r\nFailed to open binary. Check file path: "
                "'%s'\r\n\r\n",
                _filename.c_str());

        return false;
    } else {
        // Program it!
        if (verbose) printf("done\r\n");

        bool shouldProgram = true;
        // TODO: Fix memory checking
        /* if (onlyIfDifferent && */
        /*    (checkMemory(ATTINY84A_PAGESIZE, ATTINY84A_NUM_PAGES, fp, false)
         * == */
        /*     0)) */
        /*    shouldProgram = false; */

        if (!shouldProgram) {
            if (verbose)
                printf("kicker firmware is up-to-date, no need to flash\r\n");

            // exit programming mode by bringing nReset high
            exitProgramming();
        } else {
            if (verbose) printf("Starting device upload\r\n");
            bool nSuccess =
                program(fp, ATTINY84A_PAGESIZE, ATTINY84A_NUM_PAGES);

            if (verbose) printf("Device upload complete\r\n");

            if (nSuccess) {
                if (verbose) printf("FAILED\r\n");
            } else {
                if (verbose) printf("SUCCESS\r\n");
            }
        }

        fclose(fp);
    }

    return true;
}

void KickerBoard::kick(int time) {
    // TODO Replace these with constants
    int cmd = 0x3 << 6;
    transfer(cmd | time);
}

void KickerBoard::chip(int time) {
    // TODO Replace these with constants
    int cmd = 0x2 << 6;
    transfer(cmd | time);
}

uint8_t KickerBoard::read_voltage() {
    // TODO Replace these with constants
    transfer(0);
    return transfer(0);
}

uint8_t KickerBoard::transfer(const uint8_t to_send) {
    n_kick_select = !n_kick_select;
    uint8_t a = spi.write(to_send);
    n_kick_select = !n_kick_select;
    return a;
}

int KickerBoard::map(int x, int in_min, int in_max, int out_min, int out_max)
// originally an Arduino function
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
