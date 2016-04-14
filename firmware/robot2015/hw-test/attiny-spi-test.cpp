#include "mbed.h"
#include "KickerBoard.hpp"

// DigitalOut cs(p8);
Serial pc(USBTX, USBRX);  // tx and rx
LocalFileSystem local("local");


int main() {
    pc.baud(57600);  // set up the serial
    printf("About to flash kicker\r\n");
    KickerBoard kicker(p5, p6, p7, p8, p9, "/local/rj-kickr.nib");
    bool success = kicker.flash(false, true);
    printf("Flashed kicker, success = %s\r\n", success ? "TRUE" : "FALSE");

    char getCmd;

    while (1) {
        if (pc.readable()) {
            getCmd = pc.getc();
            switch (getCmd) {
                case 'k':
                    kicker.kick(20);
                    break;
                case 'c':
                    kicker.chip(8);
                    break;
                case 'r':
                    int a = kicker.read_voltage();
                    pc.printf("Received %d\r\n", a);
                    break;
            }

        }
    }
}
