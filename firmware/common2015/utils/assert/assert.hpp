#pragma once

#include <stdarg.h>

#define ASSERT_ON

/**
 * A macro for assertion checks.
 * @param e The variable to check.
 * @return Nothing is returned.
 */
#ifdef ASSERT_ON
#define ASSERT(e) \
    if (!(e)) assertFail(#e, __FILE__, __LINE__)
#else
#define ASSERT(x)
#endif

/**
 * This is called when an assertion fails.
 */
void assertFail(const char* expr, const char* file, int line);
