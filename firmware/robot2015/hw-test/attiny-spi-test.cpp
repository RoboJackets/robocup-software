
#include "mbed.h"
#include "KickerBoard.hpp"
#include "pins-ctrl-2015.hpp"
#include "SharedSPI.hpp"

// DigitalOut cs(p8);
Serial pc(USBTX, USBRX);  // tx and rx
// DigitalOut n_kick(p9);

int main() {
    pc.baud(57600);  // set up the serial
    shared_ptr<SharedSPI> sharedSPI =
        make_shared<SharedSPI>(RJ_SPI_MOSI, RJ_SPI_MISO, RJ_SPI_SCK);
    sharedSPI->format(
        8, 0);  // 8 bits per transferrintf("About to flash kicker\r\n");
    KickerBoard kickerBoard(sharedSPI, p9, p8, "/local/rj-kickr.nib");
    bool kickerReady = kickerBoard.flash(true, true);
    printf("Flashed kicker, success = %s\r\n", kickerReady ? "TRUE" : "FALSE");

    char getCmd = 'k';

    while (1) {
        //    n_kick = !n_kick;
        //    wait(1);
        if (pc.readable()) {
            getCmd = pc.getc();
            // if (getCmd == 'k') getCmd = 'c';
            // else if (getCmd == 'c') getCmd = 'r';
            // else if (getCmd == 'r') getCmd = 'k';
            pc.printf("%c: ", getCmd);
            switch (getCmd) {
                case 'k':
                    kickerBoard.kick(20);
                    break;
                case 'c':
                    kickerBoard.chip(20);
                    break;
                case 'r':
                    pc.printf("Received %d", kickerBoard.read_voltage());
                    break;
                case 'h':
                    kickerBoard.charge();
                    break;
                case 'j':
                    kickerBoard.stop_charging();
                    break;
            }
            pc.printf("\r\n");
            // wait_ms(500);
        }
    }
}
