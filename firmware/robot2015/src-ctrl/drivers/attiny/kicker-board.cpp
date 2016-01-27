#include "KickerBoard.hpp"

using namespace std;


KickerBoard::KickerBoard(PinName mosi,
    PinName miso, 
    PinName sck,
    PinName nReset,
    const string &progFilename)
        : AVR910(mosi, miso, sck, nReset)
        , _filename(progFilename)
        , _nReset(nReset)
{
}

bool KickerBoard::verify_param(const char *name, int expected, int (AVR910::*paramMethod)(), bool verbose) {
    if (verbose) printf("Checking %s...", name);
    int val = (*this.*paramMethod)();
    bool success = val == expected;
    if (verbose) {
        if (success) printf("done\r\n");
        else printf("Got unexpected value: 0x%X", val);
    }

    return success;
}

int KickerBoard::flash(bool onlyIfDifferent, bool verbose)
{
    if(verbose) printf("Enabling programming mode...");

    if( enable() != 0 ) {
        if(verbose)
            printf("failed\r\n\r\n");

        return -1;
    } else {
        if(verbose)
            printf("done\r\n\r\n");
    }


    if (!verify_param("Vendor ID", ATMEL_VENDOR_CODE, &AVR910::vendor, verbose)) return -2;


    //  Read part family and flash size - see datasheet for code meaning.
    if(verbose)
        printf("Reading Type ID...");

    int response;
    if( (response = family()) & AVR_FAMILY_MASK) {
        if(verbose)
            printf("done\r\n  AVR device architecture detected\r\n  Flash memory: %ukB\r\n\r\n", 1<<(response & 0x0F));

    } else {
        if(verbose)
            printf("done\r\n  Device architecture is not AVR\r\n\r\n");

        return -3;
    }

    if (!verify_param("Device ID", CHIP_TYPE_ATTINY13A, &AVR910::partnum, verbose)) return -4;


    //  Open binary file to write to AVR.
    if (verbose) printf("Opening kicker firmware file...");
    FILE *fp = fopen(_filename.c_str(), "r");

    if (fp == NULL) {
        if(verbose)
            printf("failed\r\nFailed to open binary. Check file path: '%s'\r\n\r\n", _filename.c_str());

        return -5;
    } else {
        //Program it!
        if(verbose)
            printf("done\r\n");


        bool shouldProgram = true;
        if (onlyIfDifferent && (verify(fp, ATTINY13A_PAGE_SIZE, ATTINY13A_NUM_PAGES) == 0)) shouldProgram = false;

        if (!shouldProgram) {
            if (verbose) printf("kicker firmware is up-to-date, no need to flash\r\n");
        } else {
            if (verbose) printf("Starting device upload\r\n");
            bool nSuccess = program(fp, ATTINY13A_PAGE_SIZE, ATTINY13A_NUM_PAGES);

            if(verbose)
                printf("Device upload complete\r\n");

            if (nSuccess) {
                if(verbose)
                    printf("FAILED\r\n");
            } else {
                if(verbose)
                    printf("SUCCESS\r\n");
            }
        }

        fclose(fp);
    }


    //  toggle the reset line to exit programming mode
    if (verbose) printf("toggling reset line to exit programming mode and reset kicker board\r\n");
    _nReset = 0;
    wait_ms(20);
    _nReset = 1;

    return 0;
}
