#include "logger.hpp"

#include <cstdarg>
#include <algorithm>

#include <mbed.h>
#include <rtos.h>

const char* LOG_LEVEL_STRING[] = {FOREACH_LEVEL(GENERATE_STRING)};

/* The initial logging level shows startup info along with any
 * warning messages [but hopefully there's none of those :) ].
 */
bool isLogging;  // = RJ_LOGGING_EN;

uint8_t rjLogLevel;

Mutex log_mutex;

class DummyStream : public std::ostream
{
public:
	template <class T>
	std::ostream& operator<<(const T&)
	{
		return *this;
	}
};

DummyStream dummy;

/**
 * [log The system-wide logging interface function. All log messages go through
 * this.]
 * @param logLevel [The "importance level" of the called log message.]
 * @param source   [The source of the message.]
 * @param format   [The string format for displaying the log message.]
 */
void log(uint8_t logLevel, const char* source, int line, const char* func,
         const char* format, ...) {
    if (isLogging && logLevel <= rjLogLevel) {
        log_mutex.lock();

        va_list args;
        char time_buf[25];
        time_t sys_time = time(NULL);
        strftime(time_buf, 25, "%H:%M:%S", localtime(&sys_time));

        va_start(args, format);

        fflush(stdout);
        printf("%s [%s] [%s:%d] <%s>\r\n  ", time_buf,
               LOG_LEVEL_STRING[logLevel], source, line, func);
        fflush(stdout);
        vprintf(format, args);
        printf("\r\n\r\n");
        fflush(stdout);

        va_end(args);
        log_mutex.unlock();
    }
}

std::ostream& log(uint8_t logLevel, const char* source, int line, const char* func) {
    if (isLogging && logLevel <= rjLogLevel) {
	char time_buf[25];
	time_t sys_time = time(NULL);
	strftime(time_buf, 25, "%H:%M:%S", localtime(&sys_time));

	std::ostream& res = std::cout << time_buf << " [" << LOG_LEVEL_STRING[logLevel] << "] [" << source << ":" << line << "] ";

	return res;
    }
    else
    {
	return dummy;
    }
}

int logLvlChange(const std::string& s) {
    int n = 0;

    n += std::count(s.begin(), s.end(), '+');
    n -= std::count(s.begin(), s.end(), '-');

    return n;
}
