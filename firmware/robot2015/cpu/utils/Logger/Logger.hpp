#ifndef LOGGER_H
#define LOGGER_H

#include "mbed.h"
#include <string>

#ifdef RJ_DEBUG_LEVEL
#undef RJ_DEBUG_LEVEL
#endif

#define RJ_DEBUG_LEVEL 0

#if RJ_DEBUG_LEVEL > 0

#define LOG(x, ...) std::printf("[LOG: %s: %u]\r\n"x"\r\n", __FILE__, __LINE__, ##__VA_ARGS__);
#define EVENT(x, ...) std::printf("[EVENT: %s: %u]\r\n"x"\r\n", __FILE__, __LINE__, ##__VA_ARGS__);
#define WARNING(x, ...) std::printf("[WARNING: %s: %u]\r\n"x"\r\n", __FILE__, __LINE__, ##__VA_ARGS__);

#else

#define LOG(x,...)
#define EVENT(x,...)
#define WARNING(x,...)

#endif

#endif  // LOGGER_H
