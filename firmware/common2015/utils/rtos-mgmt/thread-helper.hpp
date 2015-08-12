#pragma once

#include "mbed.h"
// #include "cmsis_os.h"
#include "rtos.h"

void define_thread(osThreadDef_t&, void(*task)(void const *arg), osPriority = osPriorityNormal, uint32_t = DEFAULT_STACK_SIZE, unsigned char* = NULL);
