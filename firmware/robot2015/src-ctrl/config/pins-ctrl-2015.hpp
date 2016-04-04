#pragma once

#include "PinNames.h"

// ** PIN DECLARATIONS ============================= **

// This defines the mbed pins used for the primary SPI data bus.
const PinName RJ_SPI_MOSI = p5;
const PinName RJ_SPI_MISO = p6;
const PinName RJ_SPI_SCK = p7;

// This defines the Accel/Gyro's interrupt pin.
const PinName RJ_MPU_INT = p8;

// This defines the I/O Expander's interrupt pin.
const PinName RJ_IOEXP_INT = p9;

// This defines the Radio Transceiver's interrupt pin.
const PinName RJ_RADIO_INT = p10;

// This defines the FPGA's `PROG_B` pin.
// 10k pull-up to 2.5V
const PinName RJ_FPGA_PROG_B = p11;

// This defines the robot's Green `RDY` LED. This should always be configured as
// open drain output.
const PinName RJ_RDY_LED = p12;

// This defines the FPGA's Chip Select pin. This should always be configured as
const PinName RJ_FPGA_nCS = p13;

// This defines the Kicker Board's Chip Select pin. This should always be
const PinName RJ_KICKER_nCS = p14;

// This defines the ball sensor's detector pin. This should always be configured
// analog input
const PinName RJ_BALL_DETECTOR = p15;

// This defines the Battery Voltage pin. This should always be configured as an
// analog input
const PinName RJ_BATT_SENSE = p16;

// This defines the +5V Voltage pin. This should always be configured as an
// analog input
const PinName RJ_5V_SENSE = p17;

// This defines the Piezo Speaker's pin.
const PinName RJ_SPEAKER = p18;

// This defines the FPGA's `INIT_B` pin.
// 10k pull-up to 3.3V (HSWAP_EN = NC?)
const PinName RJ_FPGA_INIT_B = p19;

// This defines one of the mbed's unused pins. It is not connected to anything
// on the 2015 Control Board rev. 1.
const PinName RJ_KICKER_nRESET = p20;

// This defines the pin used for communicating with the Mechanical Base's ID#.
const PinName RJ_BASE_ID = p21;

// This defines the radio's `TX` LED for transmitting packets. This should
// open drain
const PinName RJ_TX_LED = p22;

// This defines the radio's `RX` LED for receiving packets. This should always
// open drain
const PinName RJ_RX_LED = p23;

// This defines the `BALL` LED for ball detection. This should always be
// open drain
const PinName RJ_BALL_LED = p24;

// This defines the pin used for driving the Breakbeam's Ball Emitter LED.
const PinName RJ_BALL_EMIT = p25;

// This defines the pin used for writing color values to the `STATUS` LED.
const PinName RJ_NEOPIXEL = p26;

// This defines the set of pins used for the primary I2C data bus.
const PinName RJ_I2C_SCL = p27;
const PinName RJ_I2C_SDA = p28;

// This defines the radio transceiver's Chip Select pin. This should always be
const PinName RJ_RADIO_nCS = p29;

// This defines the FPGA's `DONE` pin.
// 300 ohm pull-up to 2.5V
const PinName RJ_FPGA_DONE = p30;

// This defines the pins used for a `Serial Connection` over the mbed's USB port
// (for use with a virtual serial connection to a computer)
#define RJ_SERIAL_RXTX MBED_UARTUSB

// ** IO-EXPANDER PINS ** ============================= **
const int RJ_IO_EXPANDER_I2C_ADDRESS = 0x42;

// Port A bit masks
const uint8_t RJ_IOEXP_A0 = 0;
const uint8_t RJ_IOEXP_A1 = 1;
const uint8_t RJ_IOEXP_A2 = 2;
const uint8_t RJ_IOEXP_A3 = 3;
const uint8_t RJ_IOEXP_A4 = 4;
const uint8_t RJ_IOEXP_A5 = 5;
const uint8_t RJ_IOEXP_A6 = 6;
const uint8_t RJ_IOEXP_A7 = 7;
// Port B bit masks
const uint8_t RJ_IOEXP_B0 = 8;
const uint8_t RJ_IOEXP_B1 = 9;
const uint8_t RJ_IOEXP_B2 = 10;
const uint8_t RJ_IOEXP_B3 = 11;
const uint8_t RJ_IOEXP_B4 = 12;
const uint8_t RJ_IOEXP_B5 = 13;
const uint8_t RJ_IOEXP_B6 = 14;
const uint8_t RJ_IOEXP_B7 = 15;

enum IOExpanderPin {
    RJ_HEX_SWITCH_BIT0 = RJ_IOEXP_A0,
    RJ_HEX_SWITCH_BIT1 = RJ_IOEXP_A1,
    RJ_HEX_SWITCH_BIT2 = RJ_IOEXP_A2,
    RJ_HEX_SWITCH_BIT3 = RJ_IOEXP_A3,
    RJ_DIP_SWITCH_1 = RJ_IOEXP_A4,
    RJ_DIP_SWITCH_2 = RJ_IOEXP_A5,
    RJ_DIP_SWITCH_3 = RJ_IOEXP_A6,
    RJ_PUSHBUTTON = RJ_IOEXP_A7,

    RJ_ERR_LED_M1 = RJ_IOEXP_B0,
    RJ_ERR_LED_M2 = RJ_IOEXP_B1,
    RJ_ERR_LED_M3 = RJ_IOEXP_B2,
    RJ_ERR_LED_M4 = RJ_IOEXP_B3,
    RJ_ERR_LED_MPU = RJ_IOEXP_B4,
    RJ_ERR_LED_BSENSE = RJ_IOEXP_B5,
    RJ_ERR_LED_DRIB = RJ_IOEXP_B6,
    RJ_ERR_LED_RADIO = RJ_IOEXP_B7,
};

constexpr uint16_t IOExpanderErrorLEDMask = 0xFF00;
