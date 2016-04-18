#include "mbed.h"
#include "KickerBoard.hpp"
#include "pins-ctrl-2015.hpp"
#include "SharedSPI.hpp"

// DigitalOut cs(p8);
Serial pc(USBTX, USBRX);  // tx and rx


int main() {
    pc.baud(57600);  // set up the serial
    shared_ptr<SharedSPI> sharedSPI =
        make_shared<SharedSPI>(RJ_SPI_MOSI, RJ_SPI_MISO, RJ_SPI_SCK);
    sharedSPI->format(8, 0);  // 8 bits per transferrintf("About to flash kicker\r\n");
    KickerBoard kickerBoard(sharedSPI, p9, p8,
                            "/local/rj-kickr.nib");
    bool kickerReady = kickerBoard.flash(true, true);
    printf("Flashed kicker, success = %s\r\n", kickerReady ? "TRUE" : "FALSE");

    char getCmd;

    while (1) {
        if (pc.readable()) {
            getCmd = pc.getc();
            switch (getCmd) {
                case 'k':
                    kickerBoard.kick(20);
                    break;
                case 'c':
                    kickerBoard.chip(8);
                    break;
                case 'r':
                    int a = kickerBoard.read_voltage();
                    pc.printf("Received %d\r\n", a);
                    break;
            }

        }
    }
}
