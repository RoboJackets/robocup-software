#pragma once

#include "mcp23017.hpp"

enum errorLED {
    ERROR_LED1 = 0x01,
    ERROR_LED2 = 0x02
};

enum DipSwitch {
    DIP_SWITCH1 = 0x01,
    DIP_SWITCH2 = 0x02
};

class IOExpander : public MCP23017
{
  public:
    IOExpander(PinName sda, PinName scl, PinName intPin);

    //~IOExpander(void);

    void Init(void);

    bool readSwitches(DipSwitch sw);
    uint8_t readHexSelector(void);

    void setErrorLED(errorLED led);
    uint8_t readLEDStates(void);

  protected:
    uint8_t robotID;
    uint8_t switchStates;

  private:
    PinName         _intPin;
    InterruptIn     _intIn;
};
