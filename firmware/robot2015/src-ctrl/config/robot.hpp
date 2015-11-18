#pragma once

// **
// =================================================================================================
// **
// **
// =================================================================================================
// **

#include "robot-devices.hpp"

// Standard library headers
#include <ctime>
#include <cstdio>

// Include the basic classes - Note: the header files included within "mbed.h"
// are listed here.
#include <mbed.h>
#include <rtos.h>

// Include the header file for the watchdog timer class
#include <watchdog.hpp>
#include <Console.hpp>

// Include the primary radio class if 915MHz band radio [if active]
#if RJ_RADIO_EN

#ifdef RJ_CC1201
#include <CC1201Radio.hpp>
#else

#ifdef RJ_CC1101
#include <CC1101.hpp>
#endif

#endif

#endif /* RJ_RADIO_EN */

#include <helper-funcs.hpp>
#include <CommModule.hpp>

#include "adc-dma.hpp"
#include "dma.hpp"
#include "controller.hpp"
#include "fpga.hpp"
