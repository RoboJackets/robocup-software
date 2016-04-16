#pragma once

#include <array>
#include <string>

/**
 * This class facilitates reading the value from a rotary selector.
 *
 * This class is templated so that it generalizes I/O functionality to not only
 * IO expander pins but also standard mBed I/O pins.
 *
 * Example usage:
 * RotarySelector<DigitalIn> selector({p5, p6, p7, p8});
 * printf("selector value: %d\r\n", selector.read());
 */
template <typename DIGITAL_IN>
class RotarySelector {
public:
    static constexpr size_t NUM_PINS = 4;

    /**
     * Initialize a RotarySelector with the given digital input pins
     *
     * @param pins An array of pins ordered from least significant to most
     */
    RotarySelector(std::array<DIGITAL_IN, NUM_PINS> pins) : _pins(pins) {}

    /**
     * Gives the reading (0x0 - 0xF) of the NUM_PINS wires
    */
    uint8_t read() {
        uint8_t reading = 0;
        for (size_t i = 0; i < NUM_PINS; i++) reading |= _pins[i].read() << i;
        return reading;
    }

    /**
     * Gives the reading (0x0 - 0xF) in string standard format.
    */
    std::string readStr() {
        char ridStr[2];
        sprintf(ridStr, "%02u", read());
        return std::string(ridStr);
    }

    /** An operator shorthand for read()
     */
    operator int() { return read(); }

private:
    std::array<DIGITAL_IN, NUM_PINS> _pins;
};
