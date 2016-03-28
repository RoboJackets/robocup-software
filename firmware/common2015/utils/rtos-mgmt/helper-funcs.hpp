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

unsigned int get_num_threads();

/**
 * Get the max amount of stack space used by a given thread.
 *
 * This was borrowed from mbed's Thread.max_stack() function but uses a P_TCB as
 * an argument rather than a Thread object.
 *
 * @return Max stack usage so far, in bytes
 */
uint32_t ThreadMaxStackUsed(const P_TCB tcb);
