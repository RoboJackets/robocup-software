#pragma once


#include "mbed.h"
#include "../utils/robot_types.hpp"


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
