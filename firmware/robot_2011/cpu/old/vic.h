#ifndef _VIC_H_
#define _VIC_H_

enum
{
    VIC_TIMER0 = 4,
    VIC_TIMER1 = 5
};

void vic_init(void);
void vic_register(int source, void (*handler)(void));

#endif // _VIC_H_
