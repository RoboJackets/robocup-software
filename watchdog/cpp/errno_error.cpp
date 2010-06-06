#include <string.h>
#include <errno.h>

#include "errno_error.h"

errno_error::errno_error()
{
    _errno = errno;
}

const char *errno_error::what() const throw()
{
    return strerror(_errno);
}
