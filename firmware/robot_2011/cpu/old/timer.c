#include "timer.h"
#include "vic.h"
#include "pins.h"
#include "lpc2103.h"

timer_t *timer_slot[MAX_TIMERS];

void set_flag(volatile void *flag)
{
    *(volatile int *)flag = 1;
}

volatile int done = 0;
timer_t delay_timer = {0, 0, set_flag, &done};

void delay_ms(int ms)
{
    delay_timer.time_left = ms;
    done = 0;
    timer_register(&delay_timer);
    while (!done)
    {
        //FIXME - Sleep?
    }
    timer_unregister(&delay_timer);
}

void irq_timer1() __attribute__((interrupt("IRQ")));
void irq_timer1()
{
    int i;
    
    for (i = 0; i < MAX_TIMERS; ++i)
    {
        timer_t *t = timer_slot[i];
        if (t && t->time_left > 0)
        {
            --t->time_left;
            
            if (t->time_left == 0)
            {
                if (t->handler)
                {
                    t->handler(t->param);
                }
                t->time_left = t->reset;
            }
        }
    }
    
    T1IR = 1;
    VICVectAddr = 0;
}

void timer_init()
{
    int i;
    
    for (i = 0; i < MAX_TIMERS; ++i)
    {
        timer_slot[i] = 0;
    }
    
    // Configure timers
    T1MR0 = 58981;      // 1ms timer
    T1MCR = 0x03;       // Reset and interrupt on match 0
    T1TCR = 2;          // Reset the timer and prescalar
    T1TCR = 1;          // Start the timer
    
    vic_register(VIC_TIMER1, irq_timer1);
}

void timer_register(timer_t *t)
{
    int i;
    for (i = 0; i < MAX_TIMERS; ++i)
    {
        if (!timer_slot[i])
        {
            timer_slot[i] = t;
            break;
        }
    }
}

void timer_unregister(timer_t *t)
{
    int i;
    for (i = 0; i < MAX_TIMERS; ++i)
    {
        if (timer_slot[i] == t)
        {
            timer_slot[i] = 0;
        }
    }
}
