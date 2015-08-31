#pragma once

#define COMPILE_PARAM_LOGGING   false

#define COMPETITION_DEPLOY      false

#define ARM_MATH_CM3

// ** ============================= ** DEBUGGING OPTIONS ** ============================= **
/*
    This will enable/disable a `Log File` that is created at the mbed's startup. The file is written
    to the mbed's onboard flash memory and can be opened for debugging purposes if it is enabled.
    Note that you may have to disconnect and reconnect the mbed's USB connection for the file to show up
    if it is plugged into a computer during the startup process.
*/
#define RJ_BOOT_LOG             false

#define RJ_FPGA_ENABLE          false

// ** ============================= ** ENABLE / DISABLE ROBOT INTERFACES ** ============================= **

// Enable/Disable the Radio Transceiver (915MHz band)
#define RJ_RADIO_EN             false
#define RJ_CC1201
//#define RJ_CC1101

// Enable/Disable the Accel/Gyro (MPU-6050)
#define RJ_MPU_EN               true

#define RJ_WATCHDOG_TIMER_EN    false

// The `Watchdog Timer` timeout value. The mbed will reset if the timer is not reset after the number of seconds defined here
#define RJ_WATCHDOG_TIMER_VALUE 2   // seconds

// The number of ms the flashing status light waits between flashes.
#define RJ_LIFELIGHT_TIMEOUT_MS 1400

// The amount of time (in ms) that all LEDs stay lit during startup
#define RJ_STARTUP_LED_TIMEOUT_MS 500

#define RJ_FPGA_SPI_FREQ        12000000

enum PORTS {
    COMM_PORT_LINK_TEST         = 0x03,
    COMM_PORT_CONTROLLER        = 0x04,
    COMM_PORT_SETPOINT          = 0x05,
    COMM_PORT_GAMEPLAY_STROBE   = 0x06,
    COMM_PORT_DISCOVERY         = 0x07,
    COMM_PORT_BULK_DATA         = 0x08
};
