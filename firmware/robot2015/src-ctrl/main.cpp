/**
 * Program an AVR with an mbed.
 */
#include "KickerBoard.hpp"

LocalFileSystem local("local");

int main() {
    Serial pc(USBTX, USBRX);
    pc.baud(57600);

    printf("About to flash kicker\r\n");
    KickerBoard kicker(p5,p6,p7,p8, "/local/rj-kickr.nib");
    bool success = kicker.flash(false, true);
    printf("Flashed kicker, success = %s\r\n", success ? "TRUE" : "FALSE");

    return 0;
}
