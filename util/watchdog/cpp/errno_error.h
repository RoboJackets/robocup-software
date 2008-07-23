#ifndef _ERRNO_ERROR_H_
#define _ERRNO_ERROR_H_

#include <stdexcept>

class errno_error: public std::exception
{
public:
    errno_error();
    
    const char *what() const throw();

protected:
    int _errno;
};

#endif // _ERRNO_ERROR_H_
