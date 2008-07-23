// Log messages are written with a timestamp at the beginning of each line.
// This is done by using fopencookie() to create a FILE* with a custom write
// function which writes the timestamp at the beginning of each line,
// so embedded \n's will still be handled correctly.

#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <time.h>

#include <string>
#include <boost/format.hpp>

#include "log.h"

using namespace std;
using namespace boost;

FILE *log_file = NULL;
FILE *log_real = NULL;
string log_filename;
int need_timestamp = 1;

__ssize_t log_write(void *cookie, const char *buf, size_t n)
{
    FILE *fp = (FILE *)cookie;
    char time_buf[32];
    struct tm tm;
    
    size_t total = 0;
    while (n)
    {
        if (need_timestamp)
        {
            need_timestamp = 0;
            
            time_t t = time(0);
            localtime_r(&t, &tm);
            strftime(time_buf, sizeof(time_buf), "%a %F %T", &tm);
            
            size_t ret = fprintf(fp, "%s ", time_buf);
            if (ret < 0)
            {
                fflush(fp);
                return ret;
            }
        }
        
        // Find the last character to print: either \n or the last character in the buffer
        const char *end = (const char *)memchr(buf, '\n', n);
        if (end)
        {
            // Print a timestamp before the next character after the '\n'
            need_timestamp = 1;
        } else {
            // Print up to the end of the buffer
            end = buf + n - 1;
        }
        
        size_t len = end - buf + 1;
        size_t ret = fwrite(buf, 1, len, fp);
        if (ret < 0)
        {
            fflush(fp);
            return ret;
        }
        
        total += ret;
        if (ret != len)
        {
            fflush(fp);
            return total;
        }
        
        n -= ret;
        buf += ret;
    }
    
    fflush(fp);
    return total;
}

int log_close(void *cookie)
{
    FILE *fp = (FILE *)cookie;
    
    if (fp == stderr)
    {
        return 0;
    } else {
        return fclose(fp);
    }
}

void log_open(const char *name)
{
    if (log_file)
    {
        fclose(log_file);
        log_file = 0;
    }

    // Open the log file
    if (name)
    {
        // Generate the complete filename
        time_t t = time(0);
        struct tm tm;
        localtime_r(&t, &tm);
        
        log_filename = str(format("/tmp/watchdog/%s-%04d-%02d-%02d-%02d:%02d:%02d.log") % name
            % (tm.tm_year + 1900) % (tm.tm_mon + 1) % tm.tm_mday % tm.tm_hour % tm.tm_min % tm.tm_sec);
    
        const char *c_name = log_filename.c_str();
        
        log_real = fopen(c_name, "wt");
        if (!log_real)
        {
            fprintf(stderr, "Watchdog: Can't write to log file %s: %m\n", c_name);
            
            log_real = stderr;
        }
    } else {
        // Write to stderr instead of a file
        log_real = stderr;
    }
    
    // Create a "cookie" FILE* for writing timestamps
    cookie_io_functions_t iof =
    {
        0,              // read
        log_write,      // write
        0,              // seek
        log_close       // close
    };
    
    log_file = fopencookie(log_real, "wt", iof);
    fflush(log_file);
}

void log_printf(const char *fmt, ...)
{
    va_list args;
    
    va_start(args, fmt);
    if (log_file)
    {
        vfprintf(log_file, fmt, args);
        fflush(log_file);
    } else {
        vfprintf(stderr, fmt, args);
        fflush(stderr);
    }
    va_end(args);
}

void log_close()
{
    if (log_file)
    {
        fclose(log_file);
        log_file = 0;
    }
}
