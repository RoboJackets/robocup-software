#include <stdarg.h>

#ifndef __ASSERT_H__
#define __ASSERT_H__

/**
 * [ASSERT A macro for assertion checks.]
 * @param  e [The variable to check.]
 * @return   [Nothing is returned.]
 */
#define ASSERT(e)  if (e) ; \
    else assertFail( #e, __FILE__, __LINE__ )

/**
 * [assertFail This is called when an assertion fails.]
 */
void assertFail(char *exp, char *file, int line);

#endif //__ASSERT_H__