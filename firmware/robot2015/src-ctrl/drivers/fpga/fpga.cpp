#include "fpga.hpp"


/**
 *
 */
FPGA::FPGA(PinName _cs, PinName _progB)
    : spi(RJ_SPI_BUS),
      cs(_cs, 1),
      progB(_progB, 1)
{
    isInit = false;

    wait(1);    //  1 second
    progB = 0;
    wait_us(80);
    progB = 1;

    //  8 bits per write, mode 3?
    spi.format(8, 3);

    //  1MHz - pretty slow, we can boost this later
    //  our max physical limit is 1/8 of the fpga system clock (18.4MHz), so 18.4/8 is the max
    spi.frequency(1000000);
}


/**
 *
 */
FPGA::~FPGA(void)
{
    isInit = false;
}


/**
 * [FPGA::Init Setup the FPGA interface]
 * @return  [The initialization error code.]
 */
bool FPGA::Init(void)
{
    FILE* fp = fopen("/local/robocup.nib", "r");

    if (fp == nullptr)
        return false;

    printf("opened file: %p\r\n", fp);

    int result = 0;
    char buf[10];

    while (true) {
        size_t bytes_read = fread(buf, 1, 1, fp);

        if (bytes_read == 0)
            break;

        result = spi.write(buf[0]); //  result should always be 0xFF since it's wired to high
    }

    fclose(fp);

    isInit = true;

    return true;
}