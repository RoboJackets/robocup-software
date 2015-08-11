#pragma once

#include "mbed.h"
#include "pins-ctrl-2015.hpp"

class FPGA
{
 public:
  FPGA(void) : spi(RJ_SPI_BUS), cs(RJ_FPGA_nCS), progB(RJ_FPGA_PROG_B) {};
  FPGA(PinName _cs, PinName _progB = RJ_FPGA_PROG_B);
  ~FPGA(void);
  bool Init(void);

 protected:

 private:
  bool isInit;
  SPI spi;
  DigitalOut cs, progB;
};
