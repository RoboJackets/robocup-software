#pragma once

#define COMPILE_PARAM_LOGGING false

#define COMPETITION_DEPLOY false

#define ARM_MATH_CM3

// ** DEBUGGING OPTIONS ** ============================= **
/*
    This will enable/disable a `Log File` that is created at the mbed's startup.
   The file is written
    to the mbed's onboard flash memory and can be opened for debugging purposes
   if it is enabled.
    Note that you may have to disconnect and reconnect the mbed's USB connection
   for the file to show up
    if it is plugged into a computer during the startup process.
*/
#define RJ_BOOT_LOG false

// ** ENABLE / DISABLE INTERFACES ** ============================= **

// Enable/Disable the Radio Transceiver (915MHz band)
#define RJ_CC1201
//#define RJ_CC1101

// The `Watchdog Timer` timeout value. The mbed will reset if the timer is not
// reset after the number of seconds defined here
#define RJ_WATCHDOG_TIMER_VALUE 2  // seconds

// The number of ms the flashing status light waits between flashes.
#define RJ_LIFELIGHT_TIMEOUT_MS 1400

// The amount of time (in ms) that all LEDs stay lit during startup
#define RJ_STARTUP_LED_TIMEOUT_MS 500
