#include "assert.hpp"

#include "mbed.h"

/**
 * This is called when an assertion fails. NOTE: THIS HAULTS ALL
 * EXECUTIONS!
 * @param exp  The expected value for the assertion check.
 * @param file The filename where the failure occured.
 * @param line The line number where the failure occured.
 */
void assertFail(const char* expr, const char* file, int line) {
    error("assertation failed: %s, file: %s, line %d \n", expr, file, line);
}
