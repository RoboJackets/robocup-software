#pragma once

#include "mbed.h"
#include <string>

void setISRPriorities(void);
std::string decode_marcstate(uint8_t);