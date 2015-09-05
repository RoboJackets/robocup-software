#pragma once

#include <cstdint>
#include <string>

void setISRPriorities(void);
void imAlive(void const*);
void strobeStatusLED(void const*);
std::string decode_marcstate(uint8_t);
