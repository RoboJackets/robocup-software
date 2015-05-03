#include <stdarg.h>

#ifndef __ASSERT_H__
#define __ASSERT_H__

#define ASSERT(e)  if (e) ; \
        else assertFail( #e, __FILE__, __LINE__ )

/**
 * Assert handler function
 */
void assertFail(char *exp, char *file, int line);

#endif //__ASSERT_H__