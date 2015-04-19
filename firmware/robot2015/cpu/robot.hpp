#pragma once

#define COMPETITION_DEPLOY 0

#define ARM_MATH_CM3

// ** ============================= ** DEBUGGING OPTIONS ** ============================= **

// Set the debugging level for compiling. Valid levels include `0`, `1`, `2`, `3`, & `4`.
#define RJ_DEBUG_LEVEL  0

/*
    This will enable/disable a `Log File` that is created at the mbed's startup. The file is written
    to the mbed's onboard flash memory and can be opened for debugging purposes if it is enabled.
    Note that you may have to disconnect and reconnect the mbed's USB connection for the file to show up
    if it is plugged into a computer during the startup process.
*/
#define RJ_BOOT_LOG             0

// This will check the mbed's firmware for the most known up-to-date version if enabled
#define RJ_CHECK_FIRMWARE       0


// ** ============================= ** ENABLE / DISABLE ROBOT INTERFACES ** ============================= **

// Enable/Disable the `Primary Radio Interface` (915MHz band).
#define RJ_PRIMARY_RADIO    1

// Enable/Disable the `Secondary Radio Interface` (2.4GHz band).
#define RJ_SECONDARY_RADIO  0

// Enable/Disable the `Motion Processor Interface` (MPU-9250).
#define RJ_MOTION_PROCESSOR 0

// The `Watchdog Timer` timeout value. The mbed will reset if the timer is not reset after the number of seconds defined here
#define RJ_WATCHDOG_TIMER_VALUE 2   // seconds


// ** ============================= ** PIN DECLARATIONS ** ============================= **

// This defines the pin used for the primary radio's `Chip Select` pin
#define RJ_PRIMARY_RADIO_CS     p9

// This defines the pin used for the primary radio's `Interrupt` pin
#define RJ_PRIMARY_RADIO_INT    p8

// This defines the pin used for the secondary radio's `Chip Enable` pin
#define RJ_SECONDARY_RADIO_CE   p10

// This defines the pin used for the secondary radio's `Chip Select` pin
#define RJ_SECONDARY_RADIO_CS   p11

// This defines the pin used for the secondary radio's `Interrupt` pin
#define RJ_SECONDARY_RADIO_IRQ  p12

// This defines the pin used for the primary radio's `GDO2` pin
#define RJ_PRIMARY_RADIO_GDO2   p13

// This defines the speaker's audio output pin
#define RJ_SPEAKER_OUT          p18

// This defines the ball sensor's detector pin
#define RJ_BALL_DETECTOR        p19

// This defines the ball sensor's emitter pin
#define RJ_BALL_EMITTER         p20

// This defines the pin used for the secondary radio's `Chip Select` pin
// #define RJ_SECONDARY_RADIO_CS   p21

// This defines the pin used for indicating mbed runtime activity
#define RJ_STATUS_LED           LED1

// These pin declarations are used for showing `TX` & `RX` radio connectivity
#define RJ_TX_LED               LED2
#define RJ_RX_LED               LED3

// This defines the pin used for a miscellaneous LED
#define RJ_MISC_LED             LED4

// This defines the pins used for a `Serial Connection` over the mbed's USB port (for use with a virtual serial connection to a computer)
#define RJ_SERIAL_RXTX          MBED_UARTUSB

// This defines the mbed pins used for its `Serial Peripheral Interface`
#define RJ_SPI_BUS              p5, p6, p7  // MOSI, MISO, SCK

// This defines the mbed pins used for its `I2C Interface`
#define RJ_I2C_BUS              p28, p27    // SDA, SCL

// This defines the mbed pin used for reading an analog voltage for the robot's battery
#define RJ_BATT_IN              p14

// This defines the mbed pin used for communicating to one of Adafruit's `Neopixel LEDs` for the primary power LED (RGB LEB)
#define RJ_POWER_LED            p16

// This defines the mbed pin used as an `Interrupt` for the MPU-9250
#define RJ_MOTION_PROCESSOR_INT p17

// This defines the mbed pin used as the 'Chip Select` pin for the MPU-9250
#define RJ_MOTION_PROCESSOR_CS  p19

// This defines the mbed pin used as the `Chip Select` pin for the Spartan-3E FPGA
#define RJ_FPGA_CS              p24

// These two (2) pins are used for configuring the FPGA upon startup
#define RJ_FPGA_FS0             p23
#define RJ_FPGA_FS1             p22

// This defines the mbed pin used as the `Chip Select` pin for the 16 pin I/O expander
#define RJ_IO_EXPANDER_CS       p29

// This defines the mbed pin used as the `Chip Select` pin for reading the kicker's voltage from an external ADC using I2C communication
#define RJ_ADC_CS               p26

// This defines the mbed pin used as the `Program` pin for the FPGA's configuration
#define RJ_FPGA_PROG            p25

// These are the leftover pins. This will be used for the finalized design - designs changes are in progress
#define RJ_UNUSED_1             p15
#define RJ_UNUSED_2             p30


// ** ============================= ** DO NOT EDIT ANYTHING BELOW HERE ** ============================= **
// ** ================================================================================================= **

// Include the basic classes - Note: the header files included within "mbed.h" are listed here.
#include "mbed.h"
#include "rtos.h"

#include "pnvicdef.hpp"
#include "reset.hpp"

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
#include "FirmwareCheck.hpp"
#endif

// Include the header file for the watchdog timer class
#include "Watchdog.hpp"

// Include the base classes for communication if a communication link is active
#if RJ_PRIMARY_RADIO | RJ_SECONDARY_RADIO
#include "CommModule.hpp"
#include "CommLink.hpp"
#endif

// Include the primary radio class if 915MHz band radio [if active]
#if RJ_PRIMARY_RADIO
#include "CC1101.hpp"
#endif

// Include the secondary radio class if 2.4GHz band radio [if active]
#if RJ_SECONDARY_RADIO
#include "nRF24L01.h"
#endif

#include "logger.hpp"

extern const PinName ds2411_pin;
