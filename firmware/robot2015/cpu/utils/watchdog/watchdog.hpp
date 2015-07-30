#include "mbed.h"


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
        LPC_WDT->WDCLKSEL = 0x1;                // Set CLK src to PCLK
        uint32_t clk = SystemCoreClock >> 4;    // WD has a fixed /4 prescaler, PCLK default is /4
        LPC_WDT->WDTC = s * (float)clk;
        LPC_WDT->WDMOD = 0x3;                   // Enabled and Reset
        Renew();
    }


    /**
     * Reset the timer to its originally set value
     */
    static void Renew(void)
    {
        LPC_WDT->WDFEED = 0xAA;
        LPC_WDT->WDFEED = 0x55;
    }
};
