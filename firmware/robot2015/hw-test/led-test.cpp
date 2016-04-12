#include "pins-ctrl-2015.hpp"
#include "mcp23017.hpp"
#include "io-expander.hpp"
#include "neostrip.hpp"
#include "RotarySelector.hpp"


int main(int argc, char** argv) {
    // Force off since the neopixel's hardware is stateless from previous
    // settings
    NeoStrip rgbLED(RJ_NEOPIXEL, 2);
    float defaultBrightness = 0.02f;
    rgbLED.brightness(3 * defaultBrightness);

    DigitalOut rdy_led(RJ_RDY_LED);

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
        const NeoColor colors[] = {
            NeoColorRed,
            NeoColorOrange,
            NeoColorYellow,
            NeoColorGreen,
            NeoColorBlue,
            NeoColorPurple,
            NeoColorWhite,
        };
        NeoColor color = colors[selector % 7];
        rgbLED.setPixel(0, color);
        rgbLED.setPixel(1, color);
        rgbLED.write();

        // LED demo - move rotary selector to control which red is lit up.
        const MCP23017::ExpPinName orderedErrLeds[]{
            RJ_ERR_LED_M1,     RJ_ERR_LED_M2,   RJ_ERR_LED_M3,
            RJ_ERR_LED_M4,     RJ_ERR_LED_MPU,  RJ_ERR_LED_DRIB,
            RJ_ERR_LED_BSENSE, RJ_ERR_LED_RADIO};

        int ledMask = 1 << orderedErrLeds[selector % 8];
        ioExpander.writeMask(~ledMask, IOExpanderErrorLEDMask);

        rdy_led = rotarySelector.read() % 2 == 0;
    }
}
