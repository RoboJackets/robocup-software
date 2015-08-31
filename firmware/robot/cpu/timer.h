#pragma once

#include <stdint.h>

// All times are in milliseconds.
// The current time is measured from initialization.

typedef struct Timer {
    // The time at which the handler will be called.
    // timer_start adds the current time to this when the timer is first added.
    // You may change this after the timer is started.
    unsigned int time;

    // If zero, this timer will be removed when it triggers.
    // If nonzero, period will be added to time when it triggers and
    // the timer will remain active.
    //
    // This may be changed by the handler to efficiently remove a recurring
    // timer in response to some condition.
    unsigned int period;

    // This is called when the time specified above is reached.
    // Be aware that this is called in a high-priority interrupt, so it
    // must take as little time as possible.  Don't forget your volatiles too!
    void (*handler)(void* arg);
    void* arg;

    // Next timer.  The list is singly-linked to save RAM.
    struct Timer* next;
} Timer;

extern Timer* first_timer;
extern volatile unsigned int current_time;

void timer_init(void);
void timer_start(Timer* t);
void timer_stop(Timer* t);
void delay_ms(int ms);

// Simple timer handlers.
// arg must point to a write_int_t
void toggle_bits(void* arg);
void write_uint(void* arg);
