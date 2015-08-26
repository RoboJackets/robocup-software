/* ----------------------------------------------------------------------------
 *         ATMEL Microcontroller Software Support 
 * ----------------------------------------------------------------------------
 * Copyright (c) 2008, Atmel Corporation
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * - Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the disclaimer below.
 *
 * Atmel's name may not be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * DISCLAIMER: THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * ----------------------------------------------------------------------------
 */

//------------------------------------------------------------------------------
/// \unit
/// !Purpose
/// 
/// Definition of AT91SAM7S-EK characteristics, AT91SAM7S-dependant PIOs and
/// external components interfacing.
/// 
/// !Contents
/// This file provide a large number of definitions, which are of three
/// different types.
///
/// PIO definitions are prefixed with #PIN_# or #PINS_#. They are to be used
/// with the pio peripheral to configure the pins required by the application.
///
/// First, additional information about the platform is provided by several
/// constants:
///    - BOARD_NAME is a string containing the board name
///    - The chip family and board (at91sam7s and at91sam7sek) are also
///      provided.
///    - BOARD_MAINOSC and BOARD_MCK contains the standard frequency for the
///      main oscillator and the master clock.
///
/// Contants prefixed with #BOARD_USB_# give information about the USB device
/// peripheral that is provided in the chip.
///
/// Defines prefixed with #PIN_# contain only one pin (and thus can be safely
/// used to initialize a single Pin instance), whereas defines starting with
/// #PINS_# contains either a single Pin instance with multiple pins inside it,
/// or a list of several Pin instances; they must be used as Pin[] array
/// initializer values, otherwise they are not safe.
///
/// Finally, some information about the flash controller is given by definitions
/// prefixed with #BOARD_FLASH_#.
//------------------------------------------------------------------------------

#ifndef BOARD_H 
#define BOARD_H

//------------------------------------------------------------------------------
//         Headers
//------------------------------------------------------------------------------

#if defined(at91sam7s321)
    #include "at91sam7s321/AT91SAM7S321.h"
#elif defined(at91sam7s64)
    #include "at91sam7s64/AT91SAM7S64.h"
#elif defined(at91sam7s128)
    #include "at91sam7s128/AT91SAM7S128.h"
#elif defined(at91sam7s256)
    #include "at91sam7s256/AT91SAM7S256.h"
#elif defined(at91sam7s512)
    #include "at91sam7s512/AT91SAM7S512.h"
#else
    #error Board does not support the specified chip.
#endif

//------------------------------------------------------------------------------
//         Global Definitions
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// Board
//------------------------------------------------------------------------------
/// Family definition.
#define at91sam7s
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// Clocks
//------------------------------------------------------------------------------
/// Frequency of the board main oscillator, in Hz.
#define BOARD_MAINOSC           18432000

/// Master clock frequency (when using board_lowlevel.c), in Hz.
#define BOARD_MCK               48000000
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// ADC
//------------------------------------------------------------------------------
/// ADC clock frequency, at 10-bit resolution (in Hz)
#define ADC_MAX_CK_10BIT         5000000
/// ADC clock frequency, at 8-bit resolution (in Hz)
#define ADC_MAX_CK_8BIT          8000000
/// Startup time max, return from Idle mode (in µs)
#define ADC_STARTUP_TIME_MAX       20
/// Track and hold Acquisition Time min (in ns)
#define ADC_TRACK_HOLD_TIME_MIN   600

//------------------------------------------------------------------------------
// USB
//------------------------------------------------------------------------------
/// Indicates the chip has a UDP controller.
#define BOARD_USB_UDP

/// Indicates the D+ pull-up is externally controlled.
#define BOARD_USB_PULLUP_EXTERNAL

/// Number of endpoints in the USB controller.
#define BOARD_USB_NUMENDPOINTS                  4

/// Returns the maximum packet size of the given endpoint.
/// \param i  Endpoint number.
/// \return Maximum packet size in bytes of endpoint. 
#define BOARD_USB_ENDPOINTS_MAXPACKETSIZE(i)    ((i == 0) ? 8 : 64)

/// Returns the number of FIFO banks for the given endpoint.
/// \param i  Endpoint number.
/// \return Number of FIFO banks for the endpoint.
#define BOARD_USB_ENDPOINTS_BANKS(i)            (((i == 0) || (i == 3)) ? 1 : 2)

/// USB attributes configuration descriptor (bus or self powered, remote wakeup)
#define BOARD_USB_BMATTRIBUTES                  USBConfigurationDescriptor_SELFPOWERED_NORWAKEUP

// Unused callbacks
#define USBDCallbacks_Resumed()
#define USBDCallbacks_Suspended()
#define USBDDriverCallbacks_ConfigurationChanged(...)
#define USBDDriverCallbacks_InterfaceSettingChanged(...)
#define USBDCallbacks_Reset(...)
#define USBD_IsHighSpeed() (0)

//------------------------------------------------------------------------------

// Pins
#define LED_RG		(1 <<  0)
#define LED_RR		(1 <<  1)
#define BALL_LED	(1 <<  2)
#define I2C_SDA		(1 <<  3)
#define I2C_SCL		(1 <<  4)
#define LED_LR		(1 <<  5)
#define IMU_INT		(1 <<  6)
#define BUZZ		(1 <<  7)
#define FS0			(1 <<  8)
#define FPGA_NCS	(1 <<  9)
#define FS1			(1 << 10)
#define FLASH_NCS	(1 << 11)
#define MISO		(1 << 12)
#define MOSI		(1 << 13)
#define SCK			(1 << 14)
#define ID2			(1 << 15)
#define NCONNECT	(1 << 16)
#define M5DIV		(1 << 17)
#define M2DIV		(1 << 18)
#define M3DIV		(1 << 19)
#define RADIO_INT	(1 << 20)
#define MCU_PROGB	(1 << 21)
#define RADIO_NCS	(1 << 22)
#define LED_RY		(1 << 23)
#define ID0			(1 << 24)
#define ID1			(1 << 25)
#define ID3			(1 << 26)
#define DP1			(1 << 27)
#define DP2			(1 << 28)
#define DP4			(1 << 29)
#define LED_LY		(1 << 30)
#define VBUS		(1 << 31)

#define LED_ALL		(LED_LY | LED_LR | LED_RG | LED_RY | LED_RR)

// NPCS numbers in the SPI controller
#define NPCS_FLASH	0
#define NPCS_FPGA	1
#define NPCS_RADIO	3

// Macros to turn LEDs on and off.  Works on multiple LEDs.
#define LED_ON(x)	{AT91C_BASE_PIOA->PIO_CODR = (x);}
#define LED_OFF(x)	{AT91C_BASE_PIOA->PIO_SODR = (x);}
#define LED_TOGGLE(x) {AT91C_BASE_PIOA->PIO_ODSR ^= (x);}

#define LED_IS_ON(x) (!(AT91C_BASE_PIOA->PIO_ODSR & (x)))

// I2C addresses
#define I2C_IMU3000	0x69
// The accelerometer is only available when the IMU3000 is in pass-through mode
#define I2C_ACCEL	0x0f

#define SWITCHES	(AT91C_BASE_PIOA->PIO_PDSR ^ (DP1 | DP2 | DP4 | ID0 | ID1 | ID2 | ID3))

//------------------------------------------------------------------------------
// Flash
//------------------------------------------------------------------------------
/// Indicates chip has an EFC.
#define BOARD_FLASH_EFC
//------------------------------------------------------------------------------

#endif //#ifndef BOARD_H
