#pragma once

#include "mbed.h"

#define WDT_BASE    LPC_WDT

/**
 * The Watchdog class is used for automatically resetting the microcontroller if it is not reset within a set time period
 */
class Watchdog
{
public:
    /**
     * Load timeout value and enable the timer
     * @param s [description]
     */
    static void Set(float s)
    {
        WDT_BASE->WDCLKSEL = 0x1;                // Set CLK src to PCLK
        uint32_t clk = SystemCoreClock >> 4;    // WD has a fixed /4 prescaler, PCLK default is /4
        WDT_BASE->WDTC = s * (float)clk;
        WDT_BASE->WDMOD = 0x3;                   // Enabled and Reset
        Renew();
    }

    /**
     * Reset the timer to its originally set value
     */
    static void Renew()
    {
        WDT_BASE->WDFEED = 0xAA;
        WDT_BASE->WDFEED = 0x55;
    }
};
