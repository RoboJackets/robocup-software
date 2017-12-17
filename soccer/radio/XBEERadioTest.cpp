#include <gtest/gtest.h>
#include "XBEERadio.hpp"

#include <iostream>

TEST(XBEERadio, Initialization) {
    XBEERadio radio;
    EXPECT_NE(&radio, nullptr);
}