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

// Enable/Disable the Radio Transceiver (915MHz band)
#define RJ_RADIO_EN             1
#define RJ_CC1201
//#define RJ_CC1101

// Enable/Disable the Accel/Gyro (MPU-6050)
#define RJ_MPU_EN               1

// The `Watchdog Timer` timeout value. The mbed will reset if the timer is not reset after the number of seconds defined here
#define RJ_WATCHDOG_TIMER_VALUE 2   // seconds


// ** ============================= ** PIN DECLARATIONS ** ============================= **

// This defines the mbed pins used for the primary SPI data bus.
#define RJ_SPI_MOSI             p5
#define RJ_SPI_MISO             p6
#define RJ_SPI_SCK              p7
#define RJ_SPI_BUS              RJ_SPI_MOSI, RJ_SPI_MISO, RJ_SPI_SCK

// This defines the Accel/Gyro's interrupt pin.
#define RJ_MPU_INT              p8

// This defines the I/O Expander's interrupt pin.
#define RJ_IOEXP_INT            p9

// This defines the Radio Transceiver's interrupt pin.
#define RJ_RADIO_INT            p10

// This defines the FPGA's `PROG_B` pin.
#define RJ_FPGA_PROG_B          p11

// This defines the robot's Green `RDY` LED. This should always be configured as an OPEN DRAIN OUTPUT.
#define RJ_RDY_LED              p12

// This defines the FPGA's Chip Select pin. This should always be configured as a DIGITAL OUTPUT.
#define RJ_FPGA_nCS             p13

// This defines the Kicker Board's Chip Select pin. This should always be configured as a DIGITAL OUTPUT.
#define RJ_KICKER_nCS           p14

// This defines the ball sensor's detector pin. This should always be configured as an ANALOG INPUT.
#define RJ_BALL_DETECTOR        p15

// This defines the Battery Voltage pin. This should always be configured as an ANALOG INPUT.
#define RJ_BATT_SENSE           p16

// This defines the +5V Voltage pin. This should always be configured as an ANALOG INPUT.
#define RJ_5V_SENSE             p17

// This defines the Piezo Speaker's pin.
#define RJ_SPEAKER              p18

// This defines the FPGA's `INIT_B` pin. This should always be configured as an OPEN DRAIN and has an external pull-up resistor.
#define RJ_FPGA_INIT_B          p19

// This defines one of the mbed's unused pins. It is not connected to anything on the 2015 Control Board rev. 1.
#define RJ_SPARE_IO             p20

// This defines the pin used for communicating with the Mechanical Base's ID#. This chip is located on the breakbeam board.
#define RJ_BASE_ID              p21

// This defines the radio's `TX` LED for transmitting packets.
#define RJ_TX_LED               p22

// This defines the radio's `RX` LED for receiving packets.
#define RJ_RX_LED               p23

// This defines the `BALL` LED for ball detection.
#define RJ_BALL_LED             p24

// This defines the pin used for driving the Breakbeam's Ball Emitter LED.
#define RJ_BALL_EMIT            p25

// This defines the pin used for writing color values to the `STATUS` LED.
#define RJ_NEOPIXEL             p26

// This defines the set of pins used for the primary I2C data bus.
#define RJ_I2C_SCL              p27
#define RJ_I2C_SDA              p28
#define RJ_I2C_BUS              RJ_I2C_SDA, RJ_I2C_SDA

// This defines the radio transceiver's Chip Select pin. This should always be configured as a DIGITAL OUTPUT.
#define RJ_RADIO_nCS            p29

// This defines the FPGA's `DONE` pin.
#define RJ_FPGA_DONE            p30

// This defines the pins used for a `Serial Connection` over the mbed's USB port (for use with a virtual serial connection to a computer)
#define RJ_SERIAL_RXTX          MBED_UARTUSB


// ** ============================= ** DO NOT EDIT ANYTHING BELOW HERE ** ============================= **
// ** ================================================================================================= **

// Include the basic classes - Note: the header files included within "mbed.h" are listed here.
#include "mbed.h"
#include "rtos.h"
#include "pnvicdef.hpp"
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


// #include "adc-dma.hpp"
// #include "speaker-dma.hpp"
#include "console.hpp"
#include "commands.hpp"
#include "logger.hpp"
#include "CC1201Radio.hpp"
#include "DS2411.hpp"
#include "git_version.hpp"
