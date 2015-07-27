#include "fpga.hpp"


/**
 * 
 */
FPGA::FPGA(void)
{
    isInit = false;
    // do stuff
    isInit = true;
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
ERR_t FPGA::Init(void)
{

}