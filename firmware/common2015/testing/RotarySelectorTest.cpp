#include <gtest/gtest.h>

#include "../../robot2015/src-ctrl/drivers/rotarySelector/RotarySelector.hpp"
#include "FakeMbed.hpp"

// A very simple test that checks that the RotarySelector calculates the value
// appropriately given a set of binary inputs.
TEST(RotarySelector, read) {
    RotarySelector<fake_mbed::DigitalIn> selector({1, 1, 0, 1});
    EXPECT_EQ(11, selector.read());
}
