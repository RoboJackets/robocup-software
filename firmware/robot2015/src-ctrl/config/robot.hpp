#pragma once

// ** ================================================================================================= **
// ** ================================================================================================= **

#include "robot-devices.hpp"
#include "robot-types.hpp"

// Include the basic classes - Note: the header files included within "mbed.h" are listed here.
#include <mbed.h>
#include <rtos.h>

// Standard library headers
#include <cstdio>
#include <ctime>

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

#endif  /* RJ_RADIO_EN */


#include "adc-dma.hpp"
#include "dma.hpp"
//#include "ds2411.hpp"
#include <helper-funcs.hpp>
#include "controller.hpp"
#include <CommModule.hpp>
#include "fpga.hpp"


#ifdef LINK_TOC_PARAMS
#include "toc.hpp"
#include "param.hpp"
#endif
