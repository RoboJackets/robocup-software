#ifndef _TIMER_H_
#define _TIMER_H_

#include <stdint.h>

#define MAX_TIMERS      16

typedef struct
{
    volatile uint16_t time_left;
    uint16_t reset;
    void (*handler)(volatile void *param);
    volatile void *param;
} timer_t;

void timer_init();
void timer_register(timer_t *t);
void timer_unregister(timer_t *t);

// Sets the int pointed to by flag to 1.
void set_flag(volatile void *flag);

// Waits for the given number of milliseconds
void delay_ms(int ms);

#endif // _TIMER_H_
