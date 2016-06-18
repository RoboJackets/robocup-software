#include <mbed.h>

#include <logger.hpp>

#include "RotarySelector.hpp"
#include "fpga.hpp"
#include "io-expander.hpp"
#include "robot-devices.hpp"

Ticker lifeLight;
DigitalOut ledOne(LED1);
DigitalOut ledTwo(LED2);

LocalFileSystem fs("local");

/**
 * timer interrupt based light flicker
 */
void imAlive() { ledOne = !ledOne; }

int main() {
    lifeLight.attach(&imAlive, 0.25);

    // A shared spi bus used for the fpga
    shared_ptr<SharedSPI> sharedSPI =
        make_shared<SharedSPI>(RJ_SPI_MOSI, RJ_SPI_MISO, RJ_SPI_SCK);
    sharedSPI->format(8, 0);  // 8 bits per transfer

    // Initialize and configure the fpga with the given bitfile
    FPGA::Instance = new FPGA(sharedSPI, RJ_FPGA_nCS, RJ_FPGA_INIT_B,
                              RJ_FPGA_PROG_B, RJ_FPGA_DONE);
    bool fpgaReady = FPGA::Instance->configure("/local/rj-fpga.nib");

    if (fpgaReady) {
        LOG(INIT, "FPGA Configuration Successful!");

    } else {
        LOG(FATAL, "FPGA Configuration Failed!");
    }

    DigitalOut rdy_led(RJ_RDY_LED, !fpgaReady);

    // Init IO Expander and turn all LEDs on.  The first parameter to config()
    // sets the first 8 lines to input and the last 8 to output.  The pullup
    // resistors and polarity swap are enabled for the 4 rotary selector lines.
    MCP23017 ioExpander(RJ_I2C_SDA, RJ_I2C_SCL, RJ_IO_EXPANDER_I2C_ADDRESS);
    ioExpander.config(0x00FF, 0x00f0, 0x00f0);
    ioExpander.writeMask(static_cast<uint16_t>(~IOExpanderErrorLEDMask),
                         IOExpanderErrorLEDMask);

    // rotary selector for setting motor velocities without needing a computer
    RotarySelector<IOExpanderDigitalInOut> rotarySelector(
        {IOExpanderDigitalInOut(&ioExpander, RJ_HEX_SWITCH_BIT0,
                                MCP23017::DIR_INPUT),
         IOExpanderDigitalInOut(&ioExpander, RJ_HEX_SWITCH_BIT1,
                                MCP23017::DIR_INPUT),
         IOExpanderDigitalInOut(&ioExpander, RJ_HEX_SWITCH_BIT2,
                                MCP23017::DIR_INPUT),
         IOExpanderDigitalInOut(&ioExpander, RJ_HEX_SWITCH_BIT3,
                                MCP23017::DIR_INPUT)});

    uint16_t duty_cycle_all = 0;

    std::vector<uint16_t> duty_cycles;
    duty_cycles.assign(5, 0);

    while (true) {
        std::vector<uint16_t> enc_deltas(5);
        FPGA::Instance->set_duty_get_enc(duty_cycles.data(), duty_cycles.size(),
                                         enc_deltas.data(),
                                         enc_deltas.capacity());

        // get a reading from the rotary selector
        const uint8_t rotary_vel = rotarySelector.read();

        // fixup the duty cycle to be centered around 0 and
        // increasing from 0 for both CW & CCW spins of the
        // rotary selector
        const uint8_t duty_cycle_multiplier =
            0x07 &
            static_cast<uint8_t>(8 - abs(8 - static_cast<int>(rotary_vel)));

        // calculate a duty cycle in steps of 73, this means max is 73 * 7 = 511
        duty_cycle_all = duty_cycle_multiplier * 73;

        // set the direction, the bit shifting should be self explanatory here
        // (that was a joke guys...calm down)
        duty_cycle_all |= (((rotary_vel & (1 << 3)) >> 3) << 9);

        // limit max value
        duty_cycle_all = std::min(duty_cycle_all, static_cast<uint16_t>(511));

        // set the duty cycle values all to our determined value according to
        // the rotary selector
        std::fill(duty_cycles.begin(), duty_cycles.end(), duty_cycle_all);

        wait_ms(3);
    }
}
