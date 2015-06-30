#ifndef THREAD_HELPER_H
#define THREAD_HELPER_H

#include "mbed.h"
#include "cmsis_os.h"

extern void define_thread(osThreadDef_t&, void(*task)(void const *arg), osPriority = osPriorityNormal, uint32_t = DEFAULT_STACK_SIZE, unsigned char* = NULL);

#endif  // THREAD_HELPER_H