#pragma once

#include "PinNames.h"

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
