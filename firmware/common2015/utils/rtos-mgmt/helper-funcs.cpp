#include "helper-funcs.hpp"

#include "mbed.h"
#include "rtos.h"

#include "logger.hpp"

/**
 * Initializes the peripheral nested vector interrupt controller (PNVIC) with
 * appropriate values. Low values have the higest priority (with system
 * interrupts having negative priority). All maskable interrupts are
 * diabled; do not intialize any interrupt frameworks before this funtion is
 * called. PNVIC interrupt priorities are independent of thread and NVIC
 * priorities.
 *
 * The PNVIC is independent of the NVIC responsible for system NMI's. The NVIC
 * is not accissable through the library, so in this context the NVIC functions
 * refer to the PNVIC. The configuration system for PNVIC priorities is strange
 * and different from X86. If you haven't already, look over
 * doc/ARM-Cortex-M_Interrupt-Priorities.pdf from RJ root and the online
 * documentation regarding Interrupt Priority Registers (IPRs) first.
 */
void setISRPriorities(void)
{
    __disable_irq();

    //set two bits for preemptive data, two bits for priority as the
    //structure for the IPRs. Grouping is global withing the IPRs so
    //this value should only be changed here.
    NVIC_SetPriorityGrouping(5);
    uint32_t priorityGrouping = NVIC_GetPriorityGrouping();

    //set preemptive priority default to 2 (0..3)
    //set priority default to 1 (0..3)
    uint32_t defaultPriority = NVIC_EncodePriority(priorityGrouping, 2, 1);

    //When the kernel initialzes the PNVIC, all ISRs are set to the
    //highest priority, making it impossible to elevate a few over
    //the rest, so the default priority is lowered globally for the
    //table first.
    //
    //Consult LPC17xx.h under IRQn_Type for PNVIC ranges, this is LPC1768
    //specific
    for (uint32_t IRQn = TIMER0_IRQn; IRQn <= CANActivity_IRQn; IRQn++)
        NVIC_SetPriority((IRQn_Type) IRQn, defaultPriority);

    ////////////////////////////////////
    //  begin raise priority section  //
    ////////////////////////////////////

    // reestablish watchdog
    NVIC_SetPriority(WDT_IRQn, NVIC_EncodePriority(priorityGrouping, 0, 0));

    // reestablish timers
    // NVIC_SetPriority(TIMER0_IRQn, NVIC_EncodePriority(priorityGrouping, 0, 1));
    // NVIC_SetPriority(TIMER1_IRQn, NVIC_EncodePriority(priorityGrouping, 0, 2));
    // NVIC_SetPriority(TIMER2_IRQn, NVIC_EncodePriority(priorityGrouping, 0, 3));
    // NVIC_SetPriority(TIMER3_IRQn, NVIC_EncodePriority(priorityGrouping, 0, 4));

    // make TIMER #2 2nd in line t the watchdog timer
    NVIC_SetPriority(TIMER2_IRQn, NVIC_EncodePriority(priorityGrouping, 0, 1));

    // this is the timer used in the mbed Ticker library
    NVIC_SetPriority(TIMER3_IRQn, NVIC_EncodePriority(priorityGrouping, 0, 2));



    ////////////////////////////////////
    //  begin lower priotity section  //
    ////////////////////////////////////

    //set UART (console) interrupts to minimal priority
    //when debugging radio and other time sensitive operations, this
    //interrupt will need to be deferred.
    NVIC_SetPriority(UART0_IRQn, NVIC_EncodePriority(priorityGrouping, 1, 0));

    NVIC_SetPriority(I2C2_IRQn, NVIC_EncodePriority(priorityGrouping, 1, 1));

    // NVIC_SetPriority(UART1_IRQn, NVIC_EncodePriority(priorityGrouping, 3, 2));
    // NVIC_SetPriority(UART2_IRQn, NVIC_EncodePriority(priorityGrouping, 3, 2));
    // NVIC_SetPriority(UART3_IRQn, NVIC_EncodePriority(priorityGrouping, 3, 1));

    // NVIC_EnableIRQ(TIMER0_IRQn);
    // NVIC_EnableIRQ(TIMER1_IRQn);
    // NVIC_EnableIRQ(TIMER2_IRQn);
    // NVIC_EnableIRQ(TIMER3_IRQn);

    __enable_irq();
}


/**
 * Timer interrupt based light flicker. If this stops, the code triggered
 * a fault.
 */
void imAlive(void const* args)
{
    DigitalOut* led = (DigitalOut*)args;

    *led = !(*led);
    Thread::wait(40);
    *led = !(*led);
}


/**
 * @brief      { Flash an LED }
 */
void strobeStatusLED(void const* args)
{
    DigitalInOut* led = (DigitalInOut*)args;

    *led = !(*led);
    Thread::wait(30);
    *led = !(*led);
}


std::string decode_marcstate(uint8_t b)
{
    std::string state;

    switch (b & 0x1F) {
    case 0x00:
        state = "SLEEP";
        break;

    case 0x01:
        state = "IDLE";
        break;

    case 0x02:
        state = "XOFF";
        break;

    case 0x03:
        state = "BIAS_SETTLE_MC";
        break;

    case 0x04:
        state = "REG_SETTLE_MC";
        break;

    case 0x05:
        state = "MANCAL";
        break;

    case 0x06:
        state = "BIAS_SETTLE";
        break;

    case 0x07:
        state = "REG_SETTLE";
        break;

    case 0x08:
        state = "STARTCAL";
        break;

    case 0x09:
        state = "BWBOOST";
        break;

    case 0x0A:
        state = "FS_LOCK";
        break;

    case 0x0B:
        state = "IFADCON";
        break;

    case 0x0C:
        state = "ENDCAL";
        break;

    case 0x0D:
        state = "RX";
        break;

    case 0x0E:
        state = "RX_END";
        break;

    case 0x0F:
        state = "RXDCM";
        break;

    case 0x10:
        state = "TXRX_SWITCH";
        break;

    case 0x11:
        state = "RX_FIFO_ERR";
        break;

    case 0x12:
        state = "FSTXON";
        break;

    case 0x13:
        state = "TX";
        break;

    case 0x14:
        state = "TX_END";
        break;

    case 0x15:
        state = "RXTX_SWITCH";
        break;

    case 0x16:
        state = "TX_FIFO_ERR";
        break;

    case 0x17:
        state = "IFADCON_TXRX";
        break;

    default:
        state = "ERROR DECODING STATE";
        break;
    }

    return state;
}
