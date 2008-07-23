#ifndef _ERRNO_EXCEPTION_HPP_
#define _ERRNO_EXCEPTION_HPP_

#include <errno.h>

#include <stdexcept>

class errno_exception: public std::runtime_error
{
public:
    errno_exception(): std::runtime_error(strerror(errno)) {}
    errno_exception(const std::string &str): std::runtime_error(str + ": " + strerror(errno)) {}
    
    const char *what() throw();
};

static inline void errno_assert(bool x)
{
    if (!x)
    {
        throw errno_exception();
    }
}

static inline void errno_assert(int x)
{
    if (x < 0)
    {
        throw errno_exception();
    }
}

static inline void errno_assert(bool x, const std::string &str)
{
    if (!x)
    {
        throw errno_exception(str);
    }
}

static inline void errno_assert(int x, const std::string &str)
{
    if (x < 0)
    {
        throw errno_exception(str);
    }
}

#endif // _ERRNO_EXCEPTION_HPP_
