#include <rtos.h>
#include <mbed.h>
#include "robot-config.hpp"
#include "pins-ctrl-2015.hpp"
#include "mcp23017.hpp"
#include "io-expander.hpp"
#include "neostrip.hpp"
#include "RotarySelector.hpp"

// For some reason, the linker fails if there is no call to Thread::wait()...
void fix() { Thread::wait(1); }

/*
 * This demo program lights up an error led on the control board corresponding
 * to the positin of the rotary selector (shell id dial).  The rotary selector
 * also controls the color of the two rgb leds.
 */
int main(int argc, char** argv) {
    // setup rgb leds
    NeoStrip rgbLED(RJ_NEOPIXEL, 2);
    float defaultBrightness = 0.02f;
    rgbLED.brightness(3 * defaultBrightness);

    // Init IO Expander and turn all LEDs on.  The first parameter to config()
    // sets the first 8 lines to input and the last 8 to output.  The pullup
    // resistors and polarity swap are enabled for the 4 rotary selector lines.
    MCP23017 ioExpander(RJ_I2C_SDA, RJ_I2C_SCL, RJ_IO_EXPANDER_I2C_ADDRESS);
    ioExpander.config(0x00FF, 0x00f0, 0x00f0);
    ioExpander.writeMask((uint16_t)~IOExpanderErrorLEDMask,
                         IOExpanderErrorLEDMask);

    // rotary selector for shell id
    RotarySelector<IOExpanderDigitalInOut> rotarySelector(
        {IOExpanderDigitalInOut(&ioExpander, RJ_HEX_SWITCH_BIT0,
                                MCP23017::DIR_INPUT),
         IOExpanderDigitalInOut(&ioExpander, RJ_HEX_SWITCH_BIT1,
                                MCP23017::DIR_INPUT),
         IOExpanderDigitalInOut(&ioExpander, RJ_HEX_SWITCH_BIT2,
                                MCP23017::DIR_INPUT),
         IOExpanderDigitalInOut(&ioExpander, RJ_HEX_SWITCH_BIT3,
                                MCP23017::DIR_INPUT)});

    while (true) {
        int selector = rotarySelector.read();

        // set rgb led color based on selector
        const std::array<NeoColor, 7> colors = {
            NeoColorRed,  NeoColorOrange, NeoColorYellow, NeoColorGreen,
            NeoColorBlue, NeoColorPurple, NeoColorWhite,
        };
        NeoColor color = colors[selector % colors.size()];
        rgbLED.setPixel(0, color);
        rgbLED.setPixel(1, color);
        rgbLED.write();

        // select red led to be lit
        const std::array<MCP23017::ExpPinName, 8> orderedErrLeds = {
            RJ_ERR_LED_M1,     RJ_ERR_LED_M2,   RJ_ERR_LED_M3,
            RJ_ERR_LED_M4,     RJ_ERR_LED_MPU,  RJ_ERR_LED_DRIB,
            RJ_ERR_LED_BSENSE, RJ_ERR_LED_RADIO};

        int ledMask = 1 << orderedErrLeds[selector % orderedErrLeds.size()];
        ioExpander.writeMask(~ledMask, IOExpanderErrorLEDMask);
    }
}
