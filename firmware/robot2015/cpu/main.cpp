#include "mbed.h"

DigitalOut myled(LED1);


LocalFileSystem local("local");


//  mosi, miso, sclk - connected to fpga
SPI spi(p5, p6, p7);
DigitalOut fpgaCs(p18); //  chip select for SPI to fpga
DigitalOut fpga_prog_b(p24);
Serial pc(USBTX, USBRX);


int main() {
    pc.printf("mbed booted!\r\n");

    fpga_prog_b = 1;
    wait(1);    //  1 second
    fpga_prog_b = 0;
    wait_us(80);
    fpga_prog_b = 1;

    //  8 bits per write, mode 3?
    spi.format(8, 3);

    //  1MHz - pretty slow, we can boost this later
    //  our max physical limit is 1/8 of the fpga system clock (18.4MHz), so 18.4/8 is the max
    spi.frequency(1000000);

    FILE *fp = fopen("/local/robocup.bit", "r");

    pc.printf("opened file: %p\r\n", fp);

    int result = 0;
    char buf[10];
    while (true) {
        size_t bytes_read = fread(buf, 1, 1, fp);
        if (bytes_read == 0) break;
        // pc.printf("writing: %d...\r\n", buf[0]);
        result = spi.write(buf[0]); //  result should always be 0xFF since it's wired to high
    }

    fclose(fp);


    pc.printf("got final byte from spi: %x\r\n", result);


    while(1) {
        myled = 1;
        wait(0.2);
        myled = 0;
        wait(0.2);
    }
}
