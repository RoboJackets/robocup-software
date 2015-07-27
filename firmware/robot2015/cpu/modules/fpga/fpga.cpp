#include "fpga.hpp"

FPGA::FPGA(void)
{
    isInit = false;
    // do stuff
    isInit = true;
}

FPGA::~FPGA(void)
{
    isInit = false;
}

ERR_t FPGA::Init(void)
{

}