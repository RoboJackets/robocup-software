#pragma once

#include <stdio.h>
#include <assert.h>

/**
Provides a "templated" implementation of a circular buffer using macros.

What's a circular buffer?
- A data structure that holds a fixed number of items.  It starts out empty and
new items can be "pushed" onto/into it.
Once it fills up, subsequent "pushes" will drop the oldest item.

What's it good for?
It's useful for storing the last N values of something.  I wrote this up to be
used to store past positions of the robot.

Example usage:
int main(int argc, char **argv) {
    CIRCBUFF_DECL(cb, int, 5);	//	make a new circular buffer
    CIRCBUFF_INIT(cb);			//	prepare it for use

    //	push the values 0..99 onto the buffer one at a time, printing the
buffer at each step
    for (int i = 0; i < 100; i++) {
        CIRCBUFF_PUSH(cb, i);	//	push the value
        CIRCBUFF_INT_PRINT(cb);	//	print
    }

    return 0;
}

*/

/**
CIRCBUFF_DECL() creates a struct with three fields:
buffer - a C array to hold the items in the circular buffer
size - the number of elements currently being stored in the buffer
capacity - the size of the C backing array
*/
#define CIRCBUFF_DECL(varName, type, cap) \
    struct {                              \
        int index;                        \
        int size;                         \
        type buffer[cap];                 \
        int capacity;                     \
    } varName;

/**
CIRCBUFF_INIT() resets the buffer and prepares it for use.
*/
#define CIRCBUFF_INIT(varName)                 \
    varName.capacity = sizeof(varName.buffer); \
    varName.size = 0;                          \
    varName.index = 0;

/**
CIRCBUFF_PUSH() adds the given value to the buffer at the front
*/
#define CIRCBUFF_PUSH(varName, val)                         \
    varName.buffer[varName.index] = val;                    \
    varName.index = (varName.index + 1) % varName.capacity; \
    if (varName.size < varName.capacity) varName.size++;

/**
CIRCBUFF_GET() retrieves a value from the buffer at the specified index
*/
#define CIRCBUFF_GET(circbuff, bufIndex, varOut)                \
    assert(bufIndex < circbuff.capacity);                       \
    varOut = circbuff.buffer[((circbuff.index - 1 - bufIndex) + \
                              circbuff.size * 10000) %          \
                             circbuff.size];

/**
CIRCBUFF_INT_PRINT() will print an integer buffer's values to stdout.  useful
for debugging.

TODO: generalize this so we can print more than just ints.
*/
#define CIRCBUFF_INT_PRINT(circbuff)                                           \
    int max_##circbuff =                                                       \
        circbuff.size < circbuff.capacity ? circbuff.size : circbuff.capacity; \
    for (int i_##circbuff = 0; i_##circbuff < max_##circbuff;                  \
         i_##circbuff++) {                                                     \
        int val;                                                               \
        CIRCBUFF_GET(circbuff, i_##circbuff, val);                             \
        printf("%d, ", val);                                                   \
    }                                                                          \
    printf("\n");
