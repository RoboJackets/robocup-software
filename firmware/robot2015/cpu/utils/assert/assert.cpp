#include "assert.h"

/**
 * [assertFail This is called when an assertion fails. NOTE: THIS HAULTS ALL EXECUTIONS!]
 * @param exp  [The expected value for the assertion check.]
 * @param file [The filename where the failure occured.]
 * @param line [The line number where the failure occured.]
 */
void assertFail(char *exp, char *file, int line)
{
    // Do stuff here to give critical error info before stopping all executions
    while (1);
}