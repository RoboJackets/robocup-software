#include <aic.h>
#include <board.h>

#include "timer.h"
#include "write.h"

Timer* first_timer;
volatile unsigned int current_time;

static void timer_interrupt() {
    unsigned int prev_time = current_time;

    // Get the number of ticks since the last interrupt
    unsigned int delta = AT91C_BASE_PITC->PITC_PIVR >> 20;

    // Clear the interrupt and update the time
    current_time += delta;

    // Run handlers for timers whose times have passed since the last interrupt
    Timer* prev = 0;
    for (Timer* t = first_timer; t; t = t->next) {
        // Look at the time from the last interrupt to the timer's deadline.
        // This accounts for multiple ticks per interrupt, even when
        // current_time wraps around.
        //
        // This test depends on unsigned values so timers in the distant future
        // don't appear
        // to have times in the past.
        if ((t->time - prev_time) <= delta) {
            t->handler(t->arg);

            if (t->period) {
                // Reset for another period
                t->time += t->period;
            } else if (prev) {
                // Remove the timer
                prev->next = t->next;
            } else {
                // Remove the timer (this is the first one)
                first_timer = t->next;
            }
        }

        prev = t;
    }
}

void timer_init() {
    AT91C_BASE_PITC->PITC_PIMR = AT91C_PITC_PITIEN | AT91C_PITC_PITEN | 2999;

    // Clear any pending interrupt
    AT91C_BASE_PITC->PITC_PIVR;

    // This is the SYSC interrupt, shared with other things we don't care about
    // yet
    AIC_ConfigureIT(1, 0, timer_interrupt);
    AIC_EnableIT(1);
}

void timer_start(Timer* t) {
    AIC_DisableIT(1);
    t->time += current_time;
    t->next = first_timer;
    first_timer = t;
    AIC_EnableIT(1);
}

void timer_stop(Timer* t) {
    AIC_DisableIT(1);
    if (t == first_timer) {
        first_timer = t->next;
    } else {
        for (Timer* prev = first_timer; prev; prev = prev->next) {
            if (prev->next == t) {
                prev->next = t->next;
                break;
            }
        }
    }
    AIC_EnableIT(1);
}

void delay_ms(int ms) {
    for (unsigned int t = current_time; current_time <= (t + ms);) {
        // Reset the watchdog timer
        AT91C_BASE_WDTC->WDTC_WDCR = 0xa5000001;
    }
}

void toggle_bits(void* arg) {
    const write_uint_t* w = (const write_uint_t*)arg;
    *w->ptr ^= w->value;
}

void write_uint(void* arg) {
    const write_uint_t* w = (const write_uint_t*)arg;
    *w->ptr = w->value;
}
