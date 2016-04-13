#pragma once

#include "PinNames.h"
#include "mcp23017.hpp"

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
constexpr int RJ_IO_EXPANDER_I2C_ADDRESS = 0x42;

constexpr auto RJ_HEX_SWITCH_BIT0 = MCP23017::PinA6;
constexpr auto RJ_HEX_SWITCH_BIT1 = MCP23017::PinA4;
constexpr auto RJ_HEX_SWITCH_BIT2 = MCP23017::PinA7;
constexpr auto RJ_HEX_SWITCH_BIT3 = MCP23017::PinA5;
constexpr auto RJ_DIP_SWITCH_1 = MCP23017::PinA4;
constexpr auto RJ_DIP_SWITCH_2 = MCP23017::PinA5;
constexpr auto RJ_DIP_SWITCH_3 = MCP23017::PinA6;
constexpr auto RJ_PUSHBUTTON = MCP23017::PinA7;

constexpr auto RJ_ERR_LED_M1 = MCP23017::PinB1;
constexpr auto RJ_ERR_LED_M2 = MCP23017::PinB0;
constexpr auto RJ_ERR_LED_M3 = MCP23017::PinB5;
constexpr auto RJ_ERR_LED_M4 = MCP23017::PinB7;
constexpr auto RJ_ERR_LED_MPU = MCP23017::PinB6;
constexpr auto RJ_ERR_LED_BSENSE = MCP23017::PinB4;
constexpr auto RJ_ERR_LED_DRIB = MCP23017::PinB3;
constexpr auto RJ_ERR_LED_RADIO = MCP23017::PinB2;

constexpr uint16_t IOExpanderErrorLEDMask = 0xFF00;
