#include "system.hpp"


/**
 *
 */
System::System(void)
{
    isInit = false;
    // do stuff
    isInit = true;
}


/**
 *
 */
System::~System(void)
{
    isInit = false;
}


/**
 * [System::Init Setup the FPGA interface]
 * @return  [The initialization error code.]
 */
ERR_t System::Init(void)
{
    if (isInit)
        return 0;

    isInit = true;
}
