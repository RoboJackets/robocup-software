#pragma once

#include <stdint.h>

static const uint16_t RJ_BASE2015_VENDOR_ID = 0x524A;
static const uint16_t RJ_BASE2015_PRODUCT_ID = 0x4253;
static const uint16_t RJ_BASE2015_RELEASE = 0x0000;

// USB "control transfer" commands recognized by the base station
enum Base2015ControlCommand {
    RadioWriteRegister = 1,
    RadioReadRegister,
    RadioStrobe,
    RadioSetChannel,
};
