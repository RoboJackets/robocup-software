#pragma once


#include "robot.hpp"


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
