#pragma once

#include <cstdarg>
#include <string>
#include <sstream>

#include "rj-macros.hpp"

// Do weird macro things for logging the filename and line for every call.
// Also allows for disabling all logging through macros so all log calls can be
// removed from production builds.
#ifdef RJ_LOGGING_EN

// Gets curent file name without path
// see http://stackoverflow.com/questions/8487986/file-macro-shows-full-path
#ifdef __FILE_NAME__
#define __BASE_FILE_NAME__                                         \
    (strrchr(__FILE_NAME__, '/') ? strrchr(__FILE_NAME__, '/') + 1 \
                                 : __FILE_NAME__)
#else  // __FILE_NAME__
#define __BASE_FILE_NAME__ \
    (strrchr(__FILE__, '/') ? (strrchr(__FILE__, '/') + 1) : __FILE__)
#endif

#define LOG(lvl, ...) \
    log(lvl, __BASE_FILE_NAME__, __LINE__, __func__, __VA_ARGS__)

#define S_LOG(lvl) \
    LogHelper(lvl, __BASE_FILE_NAME__, __LINE__, __func__)

#else             // RJ_LOGGING_EN
#define LOG(...)  // Nothing
#endif

#define FOREACH_LEVEL(LEVEL) \
    LEVEL(LOG_LEVEL_START)   \
    LEVEL(FATAL)             \
    LEVEL(SEVERE)            \
    LEVEL(WARN)              \
    LEVEL(INIT)              \
    LEVEL(OK)                \
    LEVEL(INF1)              \
    LEVEL(INF2)              \
    LEVEL(INF3)              \
    LEVEL(LOG_LEVEL_END)

/**
 * Log levels.
 */
enum LOG_LEVEL { FOREACH_LEVEL(GENERATE_ENUM) };

/**
 * Enumeration -> String conversions.
 */
extern const char* LOG_LEVEL_STRING[];

/**
 * Active logging.
 */
extern bool isLogging;

/**
 * Current log level.
 */
extern uint8_t rjLogLevel;

/**
 * [Collects the stream log message into a single string to print]
 * @param logLevel [The "importance level" of the called log message.]
 * @param source   [The source of the message.]
 * @param format   [The string format for displaying the log message.]
 */
class LogHelper : public std::stringstream {
public:
    LogHelper(uint8_t logLevel, const char* source, int line, const char* func);
    ~LogHelper();

private:
    uint8_t m_logLevel;
    const char* m_source;
    int m_line;
    const char* m_func;
}; 

/**
 * [log The system-wide logging interface function. All log messages go through
 * this.]
 * @param logLevel [The "importance level" of the called log message.]
 * @param source   [The source of the message.]
 * @param format   [The string format for displaying the log message.]
 */
void log(uint8_t logLevel, const char* source, int line, const char* func,
         const char* format, ...);

int logLvlChange(const std::string& s);
