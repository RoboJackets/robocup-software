#include "io-expander.hpp"


IOExpander::IOExpander(PinName sda, PinName scl, PinName intPin)
    : MCP23017(sda, scl), _intIn(intPin)
{
    _intPin = intPin;
    _intIn.disable_irq();   // don't trigger any interrupts until we explicitly tell it to.
}

void IOExpander::Init(void)
{
    _intIn.enable_irq();
}


bool IOExpander::readSwitches(DipSwitch sw)
{
    return false;
}


uint8_t IOExpander::readHexSelector(void)
{
    return 0x00;
}


void IOExpander::setErrorLED(errorLED led)
{
    return;
}


uint8_t IOExpander::readLEDStates(void)
{
    return 0x00;
}
