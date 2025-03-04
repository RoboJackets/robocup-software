#pragma once

#include <stdint.h>

static const uint16_t RJ_BASE_VENDOR_ID = 0x524A;
static const uint16_t RJ_BASE_PRODUCT_ID = 0x4253;
static const uint16_t RJ_BASE_RELEASE = 0x0000;

// USB "control transfer" commands recognized by the base station
enum BaseControlCommand {
    RadioWriteRegister = 1,
    RadioReadRegister,
    RadioStrobe,
    RadioSetChannel,
};
