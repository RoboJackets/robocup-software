// #pragma once


/* Macros for string conversion pass through 2 sets to help out
 * with variable expansion. Not really sure if this is needed, but
 * doing it anyways since there's pratically magic happening between
 * the pre-processor and the log function in 'logger.hpp'.
 */
#define STRINGIFY(x)        #x
#define TO_STRING(x)        STRINGIFY(x)

// These assist in making enums & strings with iterative pre-processor calls
#define GENERATE_ENUM(ENUM)     ENUM,
#define GENERATE_STRING(STRING) #STRING,
