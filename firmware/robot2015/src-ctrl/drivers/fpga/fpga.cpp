#include "fpga.hpp"

#include <logger.hpp>


bool FPGA::isInit = false;


FPGA::FPGA(PinName _mosi, PinName _miso, PinName _sck, PinName _cs, PinName _progB, PinName _initB, PinName _done)
    : spi(_mosi, _miso, _sck),
      cs(_cs, 1),
      progB(_progB, PIN_OUTPUT, OpenDrain, 0),
      initB(_initB),
      done(_done)
{
    // We force the PROG_B pin low to start a fresh configuration period in the constructor

    isInit = false;

    // However, it must be HIGH during configuration, we we bring it back HIGH here
    wait_us(50);
    // while(initB) { /* wait */ };
    progB = 1;

    //  8 bits per write, mode 3 for polarity & phase of SPI transfers
    spi.format(8, 3);

    //  1MHz - pretty slow, we can boost this later
    //  our max physical limit is 1/8 of the fpga system clock (18.4MHz), so 18.4/8 is the max
    spi.frequency(2000000);
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
bool FPGA::Init(const std::string& filename)
{
    std::string filepath = "/local/";
    filepath.append(filename);

    FILE* fp = fopen(filepath.c_str(), "r");

    if (fp == nullptr) {
        return false;
    } else {
        int result = 0;
        char buf[10];

        LOG(INIT, "Opened FPGA bitfile:\t'%p'", fp);

        while (true) {
            size_t bytes_read = fread(buf, 1, 1, fp);

            if (bytes_read == 0)
                break;

            result = spi.write(buf[0]);
        }

        fclose(fp);

        // If configuration failed, the `DONE` pin will read HIGH, so we flip it to keep things lined up with the nullptr error above.
        isInit = done;

        return isInit;
    }
}
