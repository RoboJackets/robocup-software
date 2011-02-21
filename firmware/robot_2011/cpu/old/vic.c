#include "vic.h"
#include "lpc2103.h"

void vic_init()
{
    // It looks like the VIC does not always get reset on brownout.
    // Clear all entries manually so we don't get duplicate entries later.
    
    volatile unsigned long *control = &VICVectCntl0;
    
    int slot;
    for (slot = 0; slot < 16; ++slot)
    {
        *control = 0;
        ++control;
    }
}

void vic_register(int source, void (*handler)(void))
{
    volatile unsigned long *control = &VICVectCntl0;
    
    // Find the first unused slot
    int slot;
    for (slot = 0; slot < 16; ++slot)
    {
        if (!(*control & 0x20))
        {
            // Set up this slot
            volatile unsigned long *addr = &VICVectAddr0 + slot;
            *addr = (unsigned long)handler;
            *control = 0x20 | source;
            VICIntEnable |= 1 << source;
            
            break;
        }
        
        ++control;
    }
}
