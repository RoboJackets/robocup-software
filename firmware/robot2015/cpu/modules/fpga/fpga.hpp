#pragma once

#include "mbed.h"
#include "error_type.hpp"

class FPGA
{
  public:
    FPGA(void);
    ~FPGA(void);
    ERR_t Init(void);

  protected:

  private:
    bool isInit;
};
