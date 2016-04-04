/** @file */

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

/**
 * @brief      Get the number of active threads in runtime.
 *
 * @return     Number of active threads.
 */
unsigned int get_num_threads();


/**
 * @brief      Get the max amount of stack space used by a given thread.
 *
 * This was borrowed from mbed's <a href="https://developer.mbed.org/users/mbed_official/code/mbed-rtos/docs/bdd541595fc5/classrtos_1_1Thread.html#pub-methods">Thread.max_stack()</a>
 * function, but this uses a P_TCB as an argument rather than a Thread object.
 *
 * @param tcb  The pointer for the task's struct of info.
 *
 * @return     Max stack usage so far, in bytes
 */
unsigned int ThreadMaxStackUsed(const P_TCB tcb);

/**
 * @brief      Get the currently used stack size for a thread.
 *
 * This was borrowed from mbed's <a href="https://developer.mbed.org/users/mbed_official/code/mbed-rtos/docs/bdd541595fc5/classrtos_1_1Thread.html#pub-methods">Thread.used_stack()</a>
 * function, but this uses a P_TCB as an argument rather than a Thread object.
 *
 * @param tcb  The pointer for the task's struct of info.
 *
 * @return     Currently used stack size, in bytes
 */
unsigned int ThreadNowStackUsed(const P_TCB tcb);
