#pragma once

#include "mbed.h"
#include <string>
#include <iostream>

/**
 * log levels
 */ 
enum class LOG_LEVEL : uint8_t
{
	INFO = 0,
	WARN,
	ERROR
};

/**
 * enum to string conversions
 */
extern const char* LOG_LEVEL_STRING[];

/**
 * active logging
 */
extern bool isLogging;

/**
 * current log level
 */
extern LOG_LEVEL rjLogLevel;

/**
 * Add log line with printf interface
 */
void log(LOG_LEVEL logLevel, const char *source, const char *format, ...);

/**
 * Add log line with stream interface
 */
std::ostream& log(LOG_LEVEL logLevel, const char *source = "");