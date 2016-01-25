#include "helper-funcs.hpp"

#include <vector>

#include "rtos.h"

#include "logger.hpp"
#include "assert.hpp"

// The head of the linked list of active threads
extern struct OS_XCB os_rdy;

namespace {
bool comm_led_rx_en = false;
bool comm_led_tx_en = false;
bool dir = false;
}

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
void setISRPriorities() {
    __disable_irq();

    // set two bits for preemptive data, two bits for priority as the
    // structure for the IPRs. Grouping is global withing the IPRs so
    // this value should only be changed here.
    NVIC_SetPriorityGrouping(5);
    uint32_t priorityGrouping = NVIC_GetPriorityGrouping();

    // set preemptive priority default to 2 (0..3)
    // set priority default to 1 (0..3)
    uint32_t defaultPriority = NVIC_EncodePriority(priorityGrouping, 2, 1);

    // When the kernel initialzes the PNVIC, all ISRs are set to the
    // highest priority, making it impossible to elevate a few over
    // the rest, so the default priority is lowered globally for the
    // table first.
    //
    // Consult LPC17xx.h under IRQn_Type for PNVIC ranges, this is LPC1768
    // specific
    for (uint32_t IRQn = TIMER0_IRQn; IRQn <= CANActivity_IRQn; IRQn++)
        NVIC_SetPriority((IRQn_Type)IRQn, defaultPriority);

    // reestablish watchdog
    NVIC_SetPriority(WDT_IRQn, NVIC_EncodePriority(priorityGrouping, 0, 0));

    // make TIMER #2 2nd in line t the watchdog timer
    NVIC_SetPriority(TIMER2_IRQn, NVIC_EncodePriority(priorityGrouping, 0, 1));

    // this is the timer used in the mbed Ticker library
    NVIC_SetPriority(TIMER3_IRQn, NVIC_EncodePriority(priorityGrouping, 0, 2));

    // Brown-Out Detect
    NVIC_SetPriority(BOD_IRQn, NVIC_EncodePriority(priorityGrouping, 0, 4));

    // The I2C interface that's in use
    NVIC_SetPriority(I2C2_IRQn, NVIC_EncodePriority(priorityGrouping, 1, 1));

    // The SPI interface that's in use.
    NVIC_SetPriority(SPI_IRQn, NVIC_EncodePriority(priorityGrouping, 1, 2));

    // set UART (console) interrupts to minimal priority
    // when debugging radio and other time sensitive operations, this
    // interrupt will need to be deferred.
    NVIC_SetPriority(UART0_IRQn, NVIC_EncodePriority(priorityGrouping, 1, 4));

    __enable_irq();
}

/**
 * Timer interrupt based light flicker. If this stops, the code triggered
 * a fault.
 */
void imAlive(void const* arg) {
    std::vector<DigitalOut>* leds =
        const_cast<std::vector<DigitalOut>*>(reinterpret_cast<const std::vector<DigitalOut>*>(arg));

    if (dir) {
        for (size_t i = 0; i < leds->size(); ++i) {
            leds->at(i) = !(leds->at(i));
            Thread::wait(17);
            leds->at(i) = !(leds->at(i));
        }
    } else {
        for (size_t i = leds->size(); i > 0; --i) {
            leds->at(i - 1) = !(leds->at(i - 1));
            Thread::wait(17);
            leds->at(i - 1) = !(leds->at(i - 1));
        }
    }

    dir = !dir;
}

/**
 * @brief      { Flash an LED }
 */
void strobeStatusLED(DigitalInOut& led) {
    led = !led;
    Thread::wait(15);
    led = !led;
}
void strobeStatusLED(DigitalOut& led) {
    led = !led;
    Thread::wait(15);
    led = !led;
}

// all this to flash damn leds in a multithreaded environment...wtf
void commLightsTask(DigitalInOut& led, bool tx_rx_led_en) {
    if (tx_rx_led_en == true) strobeStatusLED(led);
}
void commLightsTask(DigitalOut& led, bool tx_rx_led_en) {
    if (tx_rx_led_en == true) strobeStatusLED(led);
}

void commLightsTask_TX(void const* arg) {
    DigitalOut* led =
        const_cast<DigitalOut*>(reinterpret_cast<const DigitalOut*>(arg));
    commLightsTask(*led, comm_led_tx_en);
}
void commLightsTask_RX(void const* arg) {
    DigitalOut* led =
        const_cast<DigitalOut*>(reinterpret_cast<const DigitalOut*>(arg));
    commLightsTask(*led, comm_led_rx_en);
}

void commLightsTimeout_RX(void const* arg) { comm_led_rx_en = false; }
void commLightsTimeout_TX(void const* arg) { comm_led_tx_en = false; }

void commLightsRenew_RX() { comm_led_rx_en = true; }
void commLightsRenew_TX() { comm_led_tx_en = true; }

// returns how many active threads there are
unsigned int get_num_threads() {
    unsigned int num_threads = 0;
    P_TCB p_b = (P_TCB)&os_rdy;

    while (p_b != NULL) {
        num_threads++;
        p_b = p_b->p_lnk;
    }

    return num_threads;
}

// Helper function for creating a class that uses a
// member function for the object's thread operations.
// Take note that this allocates a stack!
void define_thread(osThreadDef_t& t, void (*task)(void const* arg),
                   osPriority priority, uint32_t stack_size) {
#ifdef CMSIS_OS_RTX
    t.pthread = task;
    t.tpriority = priority;
    t.stacksize = stack_size;
    t.stack_pointer = (uint32_t*)new unsigned char[t.stacksize];
    ASSERT(t.stack_pointer != nullptr);

#endif
}
