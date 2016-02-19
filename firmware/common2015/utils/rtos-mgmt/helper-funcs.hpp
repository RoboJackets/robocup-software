#pragma once

#include <cstdint>
#include <string>

#include <mbed.h>
#include <rtos.h>
#include <cmsis_os.h>

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
void setISRPriorities();

void imAlive(void const*);

/// Flash an LED on for a small duration, then back off
template <class IO_CLASS>
void flashLED(IO_CLASS& led, uint32_t durationMsec = 15) {
    static_assert(std::is_same<DigitalOut, IO_CLASS>::value ||
                  std::is_same<DigitalInOut, IO_CLASS>::value, "Invalid IO class for flashLED()");
    led = !led;
    Thread::wait(durationMsec);
    led = !led;
}

void commLightsTask_TX(void const*);
void commLightsTask_RX(void const*);
void commLightsTimeout_RX(void const*);
void commLightsTimeout_TX(void const*);
void commLightsRenew_RX();
void commLightsRenew_TX();
unsigned int get_num_threads();
void define_thread(osThreadDef_t&, void (*task)(void const* arg),
                   osPriority = osPriorityNormal,
                   uint32_t = DEFAULT_STACK_SIZE);
