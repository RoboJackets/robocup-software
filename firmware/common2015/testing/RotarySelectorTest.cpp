#include <gtest/gtest.h>

#include "../../robot2015/src-ctrl/drivers/rotarySelector/RotarySelector.hpp"

class FakeDigitalIn {
public:
    FakeDigitalIn(int value) : _value(value) {}

    int read() const { return _value; }

private:
    int _value;
};

// A very simple test that checks that the RotarySelector calculates the value
// appropriately given a set of binary inputs.
TEST(RotarySelector, read) {
    RotarySelector<FakeDigitalIn> selector({1, 1, 0, 1});
    EXPECT_EQ(11, selector.read());
}
