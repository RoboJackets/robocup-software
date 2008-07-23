#ifndef _LOG_H_
#define _LOG_H_

#include <stdio.h>
#include <string>

// The FILE* to write logs to.
// This is normally created by fopencookie.
extern FILE *log_file;

// The actual file underlying log_file.
extern FILE *log_real;

// The name of the log file
extern std::string log_filename;

// Opens a log file.
// <name> is the name of the process.  By convention, pass "watchdog" for the watchdog process.
//
// If a log file is already opened, it is closed.
// If <name> is NULL, logs are written to stderr.
void log_open(const char *name);

// Writes a formatted line to the log file.
void log_printf(const char *fmt, ...);

// Closes the log file.
// log_printf prints to stderr after this.
void log_close(void);

#endif // _LOG_H_
