#include "mbed.h"
#include "KickerBoard.hpp"



// DigitalOut cs(p8);
Serial pc(USBTX, USBRX); // tx and rx
LocalFileSystem local("local");


int transferAndWait(const char what, SPI& spi);
int kick( int time );
int chip( int time );
int vRead();
int map(int x, int in_min, int in_max, int out_min, int out_max);
DigitalOut n_kick_select(p9);

int main() {
    pc.baud(57600); //set up the serial
    printf("About to flash kicker\r\n");
    KickerBoard kicker(p5,p6,p7,p8, "/local/rj-kickr.nib");
    bool success = kicker.flash(false, true);
    printf("Flashed kicker, success = %s\r\n", success ? "TRUE" : "FALSE");

    n_kick_select = 0;
    uint8_t transByte;
    char getCmd;
    // Setup the spi for 8 bit data, high steady state clock,
    SPI spi(p5, p6, p7); // mosi, miso, sclk
    // mode 0 doesn't seem to work right, TODO investigate if necessary
    // we may have to use chip select to get this working properly
    spi.format(8,0); // allow system to work with arduino
    spi.frequency(8000);

    while(1) {
        if (pc.readable()) {
            getCmd = pc.getc();
            switch(getCmd){
              case 'k':
                transByte = kick(255);
              case 'c':
                transByte = chip(255);
              case 'r':
                transByte = vRead();
            }
            int a;
            transferAndWait(transByte, spi);
            a = transferAndWait('0', spi);
            pc.printf("Received:");
            pc.printf("%d\r\n", a);
            wait(.1); // tested down to .025 seconds between
        }
    }
}

int transferAndWait (const char what, SPI& spi)
{
    n_kick_select = !n_kick_select;
    wait(.1);
    int a = spi.write(what);
    wait(.020);
    n_kick_select = !n_kick_select;
    return a;
} // end transferAndWait

/*

Each of these commands generates a byte that is sent over and unpacked by the ATTiny
| Command | Value |
|<---2--->|<--6-->|

Commands:

11 kick
10 chip
01 read
00 null

swedish fish theory

*/

// map time into 0-63 range

int kick( int time )
{
  //set kick command
  int cmd = 0xC0;
  time = map(time, 0, 255, 0, 63);
  return cmd | time;
}

int chip( int time )
{
  //set chip command
  int cmd = 0xB0;
  time = map(time, 0, 255, 0, 63);
  return cmd | time;
}
int vRead()
{
  //set voltage read command
  int cmd = 0xA0;
  return cmd;
}

int map(int x, int in_min, int in_max, int out_min, int out_max)
// originally an Arduino function
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
