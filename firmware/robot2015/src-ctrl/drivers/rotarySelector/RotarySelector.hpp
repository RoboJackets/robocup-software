#pragma once

/**
 * This class facilitates reading the value from a rotary selector.
 *
 * This class is templated so that it generalizes I/O functionality to not only
 * IO expander pins but also standard mBed I/O pins.
 */
template <typename DIGITAL_IN>
class RotarySelector {
public:
    /**
     * Initialize a RotarySelector with the given digital input pins
     *
     * @param pins An array of pins ordered from least significant to most
     */
    RotarySelector::RotarySelector(std::array<DIGITAL_IN, 4> pins)
        : _pins(pins) {}

    /**
     * Gives the reading (0x0 - 0xF) of the 4 wires
    */
    uint8_t RotarySelector::read() {
        uint8_t reading = 0;
        for (size_t i = 0; i < 4; i++) reading |= _pins[i].read() << i;
        return reading;
    }

private:
    std::array<DIGITAL_IN, 4> _pins;
};
