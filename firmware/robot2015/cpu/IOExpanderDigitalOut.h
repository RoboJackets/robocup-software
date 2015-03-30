#include "mbed.h"
#include "MCP23017.h"

/*
 * Pin numbers that correspond to the MCP23017 datasheet:
 * http://ww1.microchip.com/downloads/en/DeviceDoc/21952b.pdf
 */
enum IOExpanderPin
{
    IOExpanderPinA0 = 0,
    IOExpanderPinA1,
    IOExpanderPinA2,
    IOExpanderPinA3,
    IOExpanderPinA4,
    IOExpanderPinA5,
    IOExpanderPinA6,
    IOExpanderPinA7,
    IOExpanderPinB0,
    IOExpanderPinB1,
    IOExpanderPinB2,
    IOExpanderPinB3,
    IOExpanderPinB4,
    IOExpanderPinB5,
    IOExpanderPinB6,
    IOExpanderPinB7,
};

int const NUM_EXPANDER_PINS = 16;

/*
 * A digitalOut class meant to replicate basic functionality of the mBed digitalOut
 */
class IOExpanderDigitalOut
{
private:
    MCP23017 *m_expander;
    int m_pin;
public:
    IOExpanderDigitalOut(MCP23017 *exp, IOExpanderPin pin);

    /*
     * Pulls pin low if val = 0 and pulls pin high if val >= 1
     */
    void write(int const val);

    /*
     * Returns 0 if pin is low, 1 if pin is high
     */
    int read();

    /*
     * Allows the equals operator to write to a pin
     */
    IOExpanderDigitalOut& operator= (int pin)
    {
        write(pin);
        return *this;
    }

    /*
     * Allows the equals operator to read the state of another IOExpander pin
     */
    IOExpanderDigitalOut& operator= (IOExpanderDigitalOut& rhs)
    {
        write(rhs.read());
        return *this;
    }

    /*
     * Allows the equals operator to read the state of another normal IO pin
     */
    IOExpanderDigitalOut& operator= (DigitalInOut& rhs)
    {
        write(rhs.read());
        return *this;
    }

    /*
     * Allows the pin to return its value like a simple integer variable
     */
    operator int()
    {
        return read();
    }
};