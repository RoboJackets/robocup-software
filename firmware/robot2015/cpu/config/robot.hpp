#pragma once

#define COMPETITION_DEPLOY      false

#define ARM_MATH_CM3

// ** ============================= ** DEBUGGING OPTIONS ** ============================= **

// Set the debugging level for compiling. Valid levels include `0`, `1`, `2`, `3`, & `4`.
#define RJ_LOGGING_EN           true

/*
    This will enable/disable a `Log File` that is created at the mbed's startup. The file is written
    to the mbed's onboard flash memory and can be opened for debugging purposes if it is enabled.
    Note that you may have to disconnect and reconnect the mbed's USB connection for the file to show up
    if it is plugged into a computer during the startup process.
*/
#define RJ_BOOT_LOG             false

// This will check the mbed's firmware for the most known up-to-date version if enabled
#define RJ_CHECK_FIRMWARE       false


// ** ============================= ** ENABLE / DISABLE ROBOT INTERFACES ** ============================= **

// Enable/Disable the Radio Transceiver (915MHz band)
#define RJ_RADIO_EN             true
#define RJ_CC1201
//#define RJ_CC1101

// Enable/Disable the Accel/Gyro (MPU-6050)
#define RJ_MPU_EN               true

// The `Watchdog Timer` timeout value. The mbed will reset if the timer is not reset after the number of seconds defined here
#define RJ_WATCHDOG_TIMER_VALUE 2   // seconds


// ** ============================= ** DO NOT EDIT ANYTHING BELOW HERE ** ============================= **
// ** ================================================================================================= **

// Include the basic classes - Note: the header files included within "mbed.h" are listed here.
#include "mbed.h"
#include "rtos.h"
//#include "pnvicdef.hpp"
#include "pins-ctrl-2015.hpp"
#include "robot_types.hpp"

/*
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>

#include "error.h"
#include "mbed_interface.h"

#include "DigitalIn.h"
#include "DigitalOut.h"
#include "DigitalInOut.h"
#include "BusIn.h"
#include "BusOut.h"
#include "BusInOut.h"
#include "PortIn.h"
#include "PortInOut.h"
#include "PortOut.h"
#include "AnalogIn.h"
#include "AnalogOut.h"
#include "PwmOut.h"
#include "Serial.h"
#include "SerialHalfDuplex.h"
#include "SPI.h"
#include "SPISlave.h"
#include "SPIHalfDuplex.h"
#include "I2C.h"
#include "I2CSlave.h"
#include "Ethernet.h"
#include "CAN.h"

#include "Timer.h"
#include "Ticker.h"
#include "Timeout.h"
#include "LocalFileSystem.h"
#include "InterruptIn.h"
#include "wait_api.h"
#include "rtc_time.h"
*/

// Include header file for the CMSIS Real Time Operating System
//#include "cmsis_os.h"

#if RJ_CHECK_FIRMWARE
#include "firmware-check.hpp"
#endif

// Include the header file for the watchdog timer class
#include "watchdog.hpp"

// Include the base classes for communication if a communication link is active
#if RJ_RADIO_EN
#include "CommModule.hpp"
#include "CommLink.hpp"
#endif

// Include the primary radio class if 915MHz band radio [if active]
#if RJ_RADIO_EN
#ifdef RJ_CC1201
#include "CC1201.hpp"
#else
#ifdef RJ_CC1101
#include "CC1101.hpp"
#endif
#endif
#endif


#include "adc-dma.hpp"
#include "dma.hpp"
// #include "speaker-dma.hpp"
#include "console.hpp"
#include "logger.hpp"
#include "CC1201Radio.hpp"
#include "DS2411.hpp"
#include "git_version.hpp"
#include "radio-state-decode.hpp"
#include "isr-prop.hpp"
