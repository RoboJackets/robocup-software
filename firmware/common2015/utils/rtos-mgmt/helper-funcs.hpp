#pragma once

#include <cstdint>
#include <string>

#include <cmsis_os.h>

void setISRPriorities(void);
void imAlive(void const*);
void strobeStatusLED(void const*);
void define_thread(osThreadDef_t&, void (*task)(void const* arg),
                   osPriority = osPriorityNormal,
                   uint32_t = DEFAULT_STACK_SIZE);
