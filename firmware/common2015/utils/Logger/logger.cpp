#include <stdarg.h>
#include <stdio.h>

#include "logger.hpp"

const char* LOG_LEVEL_STRING[] = {"INFO", "WARN", "ERROR"};

bool isLogging;

LOG_LEVEL rjLogLevel;

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
 * log
 */
void log(LOG_LEVEL logLevel, const char *source, const char *format, ...)
{
	if (isLogging && static_cast<int>(logLevel) >= static_cast<int>(rjLogLevel))
	{
		va_list args;
		va_start(args, format);
		printf("[%s][%s] ", LOG_LEVEL_STRING[static_cast<int>(logLevel)], source);
		vprintf(format, args);
		printf("\r\n");
		fflush(stdout);
		va_end(args);
	}
}

std::ostream& log(LOG_LEVEL logLevel, const char *source)
{
	if(isLogging && static_cast<int>(logLevel) >= static_cast<int>(rjLogLevel))
	{
		std::ostream& res = std::cout << "[" << LOG_LEVEL_STRING[static_cast<int>(logLevel)] << "]";
		if(source != '\0')
		{
			// source is not empty
			res << "[" << source << "]";
		}
		return res;
	}
	else
	{
		return dummy;
	}
}
