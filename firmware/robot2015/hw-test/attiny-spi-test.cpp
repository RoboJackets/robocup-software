#include "KickerBoard.hpp"
#include "SharedSPI.hpp"
#include "mbed.h"
#include "pins-ctrl-2015.hpp"

LocalFileSystem fs("local");

// DigitalOut cs(p8);
Serial pc(USBTX, USBRX);  // tx and rx
// DigitalOut n_kick(p9);

int main() {
    pc.baud(57600);  // set up the serial
    shared_ptr<SharedSPI> sharedSPI =
        make_shared<SharedSPI>(RJ_SPI_MOSI, RJ_SPI_MISO, RJ_SPI_SCK);
    sharedSPI->format(
        8, 0);  // 8 bits per transferrintf("About to flash kicker\r\n");
    KickerBoard kickerBoard(sharedSPI, RJ_KICKER_nCS, RJ_KICKER_nRESET,
                            "/local/rj-kickr.nib");  // nCs, nReset
    bool kickerReady = kickerBoard.flash(true, true);
    printf("Flashed kicker, success = %s\r\n", kickerReady ? "TRUE" : "FALSE");

    char getCmd = 'k';

    while (true) {
        //    n_kick = !n_kick;
        wait_ms(10);
        if (pc.readable()) {
            getCmd = pc.getc();
            // if (getCmd == 'k') getCmd = 'c';
            // else if (getCmd == 'c') getCmd = 'r';
            // else if (getCmd == 'r') getCmd = 'k';
            bool invalid = false;
            pc.printf("%c: ", getCmd);
            switch (getCmd) {
                case 'k':
                    pc.printf("Resp: 0x%02X", kickerBoard.kick(240));
                    break;
                case 'c':
                    pc.printf("Resp: 0x%02X", kickerBoard.chip(240));
                    break;
                case 'r':
                    pc.printf("Volts: %d", kickerBoard.read_voltage());
                    break;
                case 'h':
                    pc.printf("Resp: 0x%02X", kickerBoard.charge());
                    break;
                case 'j':
                    pc.printf("Resp: 0x%02X", kickerBoard.stop_charging());
                    break;
                case 'p':
                    pc.printf("Resp: 0x%02X", kickerBoard.is_pingable());
                    break;
                case '1':
                    pc.printf("Kick Resp: 0x%02X",
                              kickerBoard.is_kick_debug_pressed());
                    break;
                case '2':
                    pc.printf("Chip Resp: 0x%02X",
                              kickerBoard.is_chip_debug_pressed());
                    break;
                case '3':
                    pc.printf("Charge Resp: 0x%02X",
                              kickerBoard.is_charge_debug_pressed());
                    break;
                default:
                    invalid = true;
                    pc.printf("Invalid command");
                    break;
            }

            // wait_ms(2);

            // if (!invalid) {
            //    pc.printf("\t[charging %s]",
            //           kickerBoard.is_charge_enabled() ? "ACTIVE" :
            //           "inactive");
            // }

            if (!invalid) {
                pc.printf("\t[charging %s]", kickerBoard.is_charge_enabled()
                                                 ? "ACTIVE"
                                                 : "inactive");
            }

            pc.printf("\r\n");
            fflush(stdout);

            wait_ms(2);
        }
    }
}
