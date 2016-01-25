#include "logger.hpp"

#include <cstdarg>
#include <algorithm>
#include <string>

#include <mbed.h>
#include <rtos.h>

const char* LOG_LEVEL_STRING[] = {FOREACH_LEVEL(GENERATE_STRING)};

/* The initial logging level shows startup info along with any
 * warning messages [but hopefully there's none of those :) ].
 */
volatile bool isLogging;  // = RJ_LOGGING_EN;

volatile uint8_t rjLogLevel;


namespace {
Mutex log_mutex;

std::string& basename(std::string& str)
{
    std::size_t found = str.find_last_of("/\\");
    str = str.substr(found + 1);
    return str;
}
}

/**
 * [log The system-wide logging interface function. All log messages go through
 * this.]
 * @param logLevel [The "importance level" of the called log message.]
 * @param source   [The source of the message.]
 * @param format   [The string format for displaying the log message.]
 */
void log(uint8_t logLevel, const char* source, const char* func,
         const char* format, ...) {
    if (isLogging && logLevel <= rjLogLevel) {
        log_mutex.lock();
        std::string src(source);

        va_list args;
        va_start(args, format);

        fflush(stdout);
        printf("\033[K[%s] [%s] <%s>\033E  ", LOG_LEVEL_STRING[logLevel], basename(src).c_str(), func);
        fflush(stdout);
        vprintf(format, args);
        printf("\r\n\r\n");
        fflush(stdout);

        va_end(args);
        log_mutex.unlock();
    }
}

int logLvlChange(const std::string& s) {
    int n = 0;

    n += std::count(s.begin(), s.end(), '+');
    n -= std::count(s.begin(), s.end(), '-');

    return n;
}
