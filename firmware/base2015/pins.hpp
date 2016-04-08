#pragma once

#include "PinNames.h"

// SPI bus
const PinName RJ_SPI_MOSI = p5;
const PinName RJ_SPI_MISO = p6;
const PinName RJ_SPI_SCK = p7;

// cc1201 radio chip select (inverted) and interrupt pins
const PinName RJ_RADIO_nCS = p29;
const PinName RJ_RADIO_INT = p10;

// This defines the pins used for a `Serial Connection` over the mbed's USB port
// (for use with a virtual serial connection to a computer)
#define RJ_SERIAL_RXTX MBED_UARTUSB
