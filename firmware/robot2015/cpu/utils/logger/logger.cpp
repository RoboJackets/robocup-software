#include <stdarg.h>
#include <stdio.h>

#include "logger.hpp"

const char* LOG_LEVEL_STRING[] = { FOREACH_LEVEL(GENERATE_STRING) };

/**
 * active logging
 */
volatile bool isLogging;

/**
 * current log level
 */
volatile uint8_t rjLogLevel;

/**
 * log
 */
void log(uint8_t logLevel, const char* source, const char* format, ...)
{
	if (isLogging && logLevel <= rjLogLevel)
	{
		va_list args;
		va_start(args, format);
		printf("[%s][%s] ", LOG_LEVEL_STRING[logLevel], source);
		vprintf(format, args);
		printf("\r\n");
		fflush(stdout);
		va_end(args);
	}
}