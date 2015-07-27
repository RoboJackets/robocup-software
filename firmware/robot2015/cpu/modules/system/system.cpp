#include "system.hpp"

System::System(void)
{
    isInit = false;
    // do stuff
    isInit = true;
}

System::~System(void)
{
    isInit = false;
}

ERR_t System::Init(void)
{
    if (isInit)
        return 0;

    isInit = true;
}

