#pragma once

#include <cstdint>
#include <string>

#include "mbed.h"
#include "cmsis_os.h"

void setISRPriorities(void);
void imAlive(void const*);
void strobeStatusLED(DigitalInOut&);
void strobeStatusLED(DigitalOut&);
void commLightsTask(DigitalInOut&, bool);
void commLightsTask(DigitalOut&, bool);
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
