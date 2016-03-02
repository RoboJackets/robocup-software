#include "mbed.h"
#include "KickerBoard.hpp"



// DigitalOut cs(p8);
Serial pc(USBTX, USBRX); // tx and rx
LocalFileSystem local("local");
int t = 0;
bool btnState = 0;
bool btnRead;

int transferAndWait(const char what, SPI& spi);


int main() {
    pc.baud(57600); //set up the serial
    printf("About to flash kicker\r\n");
    KickerBoard kicker(p5,p6,p7,p8, "/local/rj-kickr.nib");
    bool success = kicker.flash(false, true);
    printf("Flashed kicker, success = %s\r\n", success ? "TRUE" : "FALSE");

    DigitalOut n_kick_select(p9);
    n_kick_select = 0;
    // Setup the spi for 8 bit data, high steady state clock,
    SPI spi(p5, p6, p7); // mosi, miso, sclk
    // mode 0 doesn't seem to work right, TODO investigate if necessary
    // we may have to use chip select to get this working properly
    spi.format(8,0); // allow system to work with arduino
    spi.frequency(8000);

    while(1) {
        if (pc.readable()) {
            pc.getc();
            n_kick_select = !n_kick_select;
        }
        int a;
        a = transferAndWait(255, spi);
        pc.printf("Received:");
        pc.printf("%d\r\n", a);
        wait(.1); // tested down to .025 seconds between
    }
}

int transferAndWait (const char what, SPI& spi)
{
    int a = spi.write(what);
    wait(.020);
    return a;
} // end transferAndWait
