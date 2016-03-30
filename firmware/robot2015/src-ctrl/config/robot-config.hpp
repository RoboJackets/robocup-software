#pragma once

#define ARM_MATH_CM3

// The `Watchdog Timer` timeout value. The mbed will reset if the timer is not
// reset after the number of seconds defined here
#define RJ_WATCHDOG_TIMER_VALUE 2  // seconds

// The number of ms the flashing status light waits between flashes.
#define RJ_LIFELIGHT_TIMEOUT_MS 1400

// The amount of time (in ms) that all LEDs stay lit during startup
#define RJ_STARTUP_LED_TIMEOUT_MS 500
