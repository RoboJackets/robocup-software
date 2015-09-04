#pragma once

#include <mbed.h>
#include <string>

#include "pins-ctrl-2015.hpp"

class FPGA
{
  public:
    FPGA(void) : spi(RJ_SPI_BUS), cs(RJ_FPGA_nCS, 1), progB(RJ_FPGA_PROG_B, PIN_OUTPUT, OpenDrain, 1), initB(RJ_FPGA_INIT_B), done(RJ_FPGA_DONE) { };
    FPGA(PinName _mosi, PinName _miso, PinName _sck, PinName _cs, PinName _progB, PinName _initB, PinName _done);
    ~FPGA(void);

    bool Init(const std::string& filepath);

  private:
    static bool     isInit;

    bool send_config(const std::string& filepath);

    SPI             spi;
    DigitalOut      cs;
    DigitalInOut    progB;
    DigitalIn       initB, done;
};
