#include "helper-funcs.hpp"

#include <vector>
#include <type_traits>
#include <string>
#include <map>

#include "logger.hpp"
#include "assert.hpp"

// The head of the linked list of active threads
extern struct OS_XCB os_rdy;

std::map<std::string, std::string> sys_env;
Mutex env_mutex;

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

// returns how many active threads there are
unsigned int get_num_threads() {
    unsigned int num_threads = 0;
    P_TCB p_b = (P_TCB)&os_rdy;

    while (p_b != nullptr) {
        num_threads++;
        p_b = p_b->p_lnk;
    }

    return num_threads;
}

unsigned int ThreadMaxStackUsed(const P_TCB tcb) {
#ifndef __MBED_CMSIS_RTOS_CA9
    uint32_t high_mark = 0;
    while (tcb->stack[high_mark] == 0xE25A2EA5) high_mark++;
    return tcb->priv_stack - (high_mark * 4);
#else
    return 0;
#endif
}

unsigned int ThreadNowStackUsed(const P_TCB tcb) {
#ifndef __MBED_CMSIS_RTOS_CA9
    uint32_t top = reinterpret_cast<uint32_t>(tcb->stack) + tcb->priv_stack;
    return top - tcb->tsk_stack;
#else
    return 0;
#endif
}

int rj_putenv(const std::string& s) {
    size_t pos = 0;
    std::string s_copy(s);
    std::vector<std::string> vals;
    env_mutex.lock();
    while ((pos = s_copy.find('=')) != std::string::npos) {
        std::string token = s_copy.substr(0, pos);
        vals.push_back(token);
        s_copy.erase(0, pos + 1);
    }
    if (s_copy.size()) vals.push_back(s_copy);

    // must have exactly 2 values
    //   ex: 'x=y' goes to 'x' and 'y'
    if (vals.size() != 2) {
        env_mutex.unlock();
        return 1;
    }

    // set the mapped value in the sys_env container and return
    sys_env[vals.at(0)] = vals.at(1);
    env_mutex.unlock();
    return 0;
}

std::string rj_getenv(const std::string& name) {
    env_mutex.lock();
    if (sys_env.find(name) == sys_env.end()) {
        // not found
        env_mutex.unlock();
        return std::string("");
    }
    // found
    env_mutex.unlock();
    return sys_env.find(name)->second;
}

void rj_printenv() {
    env_mutex.lock();
    for (std::map<std::string, std::string>::iterator it = sys_env.begin();
         it != sys_env.end(); ++it)
        printf("%s=%s\r\n", it->first.c_str(), it->second.c_str());
    env_mutex.unlock();
}
