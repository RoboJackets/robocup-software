#pragma once

#include "mbed.h"
#include <string>

#define FOREACH_LEVEL(LEVEL) \
	LEVEL(FATAL)  \
	LEVEL(SEVERE) \
	LEVEL(WARN)   \
	LEVEL(OK)     \
	LEVEL(INF1)   \
	LEVEL(INF2)   \
	LEVEL(INF3)
#define GENERATE_ENUM(ENUM) ENUM,
#define GENERATE_STRING(STRING) #STRING,

/**
 * log levels
 */ 
enum LOG_LEVEL
{
	FOREACH_LEVEL(GENERATE_ENUM)
};

/**
 * enum to string conversions
 */
extern const char* LOG_LEVEL_STRING[];

/**
 * active logging
 */
extern volatile bool isLogging;

/**
 * current log level
 */
extern volatile uint8_t rjLogLevel;

/**
 * log(level, class/file, printf args)
 */
extern void log(uint8_t logLevel, const char* source, const char* format, ...);
