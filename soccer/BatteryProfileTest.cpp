#include <gtest/gtest.h>
#include "BatteryProfile.hpp"

TEST(BatteryProfile, getChargeLevel) {
    BatteryProfile profile({{1, 0.00}, {4, 0.50}, {5, 0.75}, {7, 1.00}});

    EXPECT_FLOAT_EQ(0, profile.getChargeLevel(1));
    EXPECT_FLOAT_EQ(0, profile.getChargeLevel(-100));
    EXPECT_FLOAT_EQ(1, profile.getChargeLevel(100));
    EXPECT_FLOAT_EQ((0.5 + 0.75) / 2, profile.getChargeLevel(4.5));
}
