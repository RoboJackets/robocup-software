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
 * Log levels.
 */
enum LOG_LEVEL {
    FOREACH_LEVEL(GENERATE_ENUM)
};


/**
 * Enumeration -> String conversions.
 */
extern const char *LOG_LEVEL_STRING[];


/**
 * Active logging.
 */
extern volatile bool isLogging;


/**
 * Current log level.
 */
extern volatile uint8_t rjLogLevel;


/**
 * [log The system-wide logging interface function. All log messages go through this.]
 * @param logLevel [The "importance level" of the called log message.]
 * @param source   [The source of the message.]
 * @param format   [The string format for displaying the log message.]
 */
extern void log(uint8_t logLevel, const char *source, const char *format, ...);
