#include <stdarg.h>
#include <stdio.h>
#include "logger.hpp"


const char *LOG_LEVEL_STRING[] = { FOREACH_LEVEL(GENERATE_STRING) };


/**
 * Active logging.
 */
volatile bool isLogging;


/**
 * Current log level.
 */
volatile uint8_t rjLogLevel;


/**
 * [log The system-wide logging interface function. All log messages go through this.]
 * @param logLevel [The "importance level" of the called log message.]
 * @param source   [The source of the message.]
 * @param format   [The string format for displaying the log message.]
 */
void log(uint8_t logLevel, const char *source, const char *format, ...)
{
	if (isLogging && logLevel <= rjLogLevel) {
		va_list args;
		va_start(args, format);
		printf("[%s][%s] ", LOG_LEVEL_STRING[logLevel], source);
		vprintf(format, args);
		printf("\r\n");
		fflush(stdout);
		va_end(args);
	}
}