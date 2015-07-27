/*
 * @brief LPC8xx IOCON driver
 *
 * @note
 * Copyright(C) NXP Semiconductors, 2012
 * All rights reserved.
 *
 * @par
 * Software that is described herein is for illustrative purposes only
 * which provides customers with programming information regarding the
 * LPC products.  This software is supplied "AS IS" without any warranties of
 * any kind, and NXP Semiconductors and its licenser disclaim any and
 * all warranties, express or implied, including all implied warranties of
 * merchantability, fitness for a particular purpose and non-infringement of
 * intellectual property rights.  NXP Semiconductors assumes no responsibility
 * or liability for the use of the software, conveys no license or rights under any
 * patent, copyright, mask work right, or any other intellectual property rights in
 * or to any products. NXP Semiconductors reserves the right to make changes
 * in the software without notification. NXP Semiconductors also makes no
 * representation or warranty that such application will be suitable for the
 * specified use without further testing or modification.
 *
 * @par
 * Permission to use, copy, modify, and distribute this software and its
 * documentation is hereby granted, under NXP Semiconductors' and its
 * licensor's relevant copyrights in the software, without fee, provided that it
 * is used in conjunction with NXP Semiconductors microcontrollers.  This
 * copyright, permission, and disclaimer notice must appear in all copies of
 * this code.
 */

#include "chip.h"

/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/

/* This array maps the address offsets (from 0x40044000) to the actual pin
 * in indexed order by pin number.
 */
const static uint8_t PIN_ADDRESS_OFFSETS[] = {0x44,		/*!< PIO0_0  */
											  0x2C,		/*!< PIO0_1  */
											  0x18,		/*!< PIO0_2  */
											  0x14,		/*!< PIO0_3  */
											  0x10,		/*!< PIO0_4  */
											  0x0C		/*!< PIO0_5  */
#if !defined(PKG_DIP8)
											  ,

											  0x40,		/*!< PIO0_6  */
											  0x3C,		/*!< PIO0_7  */
											  0x38,		/*!< PIO0_8  */
											  0x34,		/*!< PIO0_9  */
											  0x20,		/*!< PIO0_10 */
											  0x1C,		/*!< PIO0_11 */
											  0x08,		/*!< PIO0_12 */
											  0x04		/*!< PIO0_13 */
#if !defined(PKG_TSSOP16)
											  ,

											  0x48,		/*!< PIO0_14 */
											  0x28,		/*!< PIO0_15 */
											  0x24,		/*!< PIO0_16 */
											  0x00		/*!< PIO0_17 */
#endif
#endif
};

#define PIN_OFFSET(pin)                     (PIN_ADDRESS_OFFSETS[pin])

#define PIN_BASE_ADDR                       (0x40044000)

#define PIN_MODE_GET(pin)                   (*(volatile uint32_t *) (PIN_BASE_ADDR + PIN_OFFSET(pin)) &    (3 <<  3))
#define PIN_MODE_CLR(pin)                   (*(volatile uint32_t *) (PIN_BASE_ADDR + PIN_OFFSET(pin)) &= ~(3 <<  3))
#define PIN_MODE_SET(pin, val)              (*(volatile uint32_t *) (PIN_BASE_ADDR + PIN_OFFSET(pin)) |= (val <<  3))

#define PIN_HYS_GET(pin)                    (*(volatile uint32_t *) (PIN_BASE_ADDR + PIN_OFFSET(pin)) &   ( 1 <<  5))
#define PIN_HYS_CLR(pin)                    (*(volatile uint32_t *) (PIN_BASE_ADDR + PIN_OFFSET(pin)) &= ~(1 <<  5))
#define PIN_HYS_SET(pin, val)               (*(volatile uint32_t *) (PIN_BASE_ADDR + PIN_OFFSET(pin)) |= (val <<  5))

#define PIN_INV_GET(pin)                    (*(volatile uint32_t *) (PIN_BASE_ADDR + PIN_OFFSET(pin)) &   ( 1 <<  6))
#define PIN_INV_CLR(pin)                    (*(volatile uint32_t *) (PIN_BASE_ADDR + PIN_OFFSET(pin)) &= ~( 1 <<  6))
#define PIN_INV_SET(pin, val)               (*(volatile uint32_t *) (PIN_BASE_ADDR + PIN_OFFSET(pin)) |= (val <<  6))

#define PIN_OD_GET(pin)                     (*(volatile uint32_t *) (PIN_BASE_ADDR + PIN_OFFSET(pin)) &   ( 1 << 10))
#define PIN_OD_CLR(pin)                     (*(volatile uint32_t *) (PIN_BASE_ADDR + PIN_OFFSET(pin)) &= ~( 1 << 10))
#define PIN_OD_SET(pin, val)                (*(volatile uint32_t *) (PIN_BASE_ADDR + PIN_OFFSET(pin)) |= (val << 10))

#define PIN_SMODE_GET(pin)                  (*(volatile uint32_t *) (PIN_BASE_ADDR + PIN_OFFSET(pin)) &   ( 3 << 11))
#define PIN_SMODE_CLR(pin)                  (*(volatile uint32_t *) (PIN_BASE_ADDR + PIN_OFFSET(pin)) &= ~( 3 << 11))
#define PIN_SMODE_SET(pin, val)             (*(volatile uint32_t *) (PIN_BASE_ADDR + PIN_OFFSET(pin)) |= (val << 11))

#define PIN_CLKDIV_GET(pin)                 (*(volatile uint32_t *) (PIN_BASE_ADDR + PIN_OFFSET(pin)) &    ( 7 << 13))
#define PIN_CLKDIV_CLR(pin)                 (*(volatile uint32_t *) (PIN_BASE_ADDR + PIN_OFFSET(pin)) &=  ~( 7 << 13))
#define PIN_CLKDIV_SET(pin, val)            (*(volatile uint32_t *) (PIN_BASE_ADDR + PIN_OFFSET(pin)) |= (val << 13))

#define PIN_I2CMODE_GET(pin)                (*(volatile uint32_t *) (PIN_BASE_ADDR + PIN_OFFSET(pin)) &    ( 3 <<  8))
#define PIN_I2CMODE_CLR(pin)                (*(volatile uint32_t *) (PIN_BASE_ADDR + PIN_OFFSET(pin)) &=  ~( 3 <<  8))
#define PIN_I2CMODE_SET(pin, val)           (*(volatile uint32_t *) (PIN_BASE_ADDR + PIN_OFFSET(pin)) |= (val <<  8))

/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/

/*****************************************************************************
 * Private functions
 ****************************************************************************/

/*****************************************************************************
 * Public functions
 ****************************************************************************/

/* Set the pin mode (pull-up/pull-down). */
void Chip_IOCON_PinSetMode(LPC_IOCON_T *pIOCON, CHIP_PINx_T pin, CHIP_PIN_MODE_T mode)
{
	switch (pin) {
	case PIO10:	/* No mode for the I2C pins */
	case PIO11:
		break;

	default:
		PIN_MODE_CLR(pin);
		PIN_MODE_SET(pin, mode);
		break;
	}
}

/* Enables/disables the pin hysteresis. */
void Chip_IOCON_PinSetHysteresis(LPC_IOCON_T *pIOCON, CHIP_PINx_T pin, bool enable)
{
	switch (pin) {
	case PIO10:	/* No hysteresis for the I2C pins */
	case PIO11:
		break;

	default:
		PIN_HYS_CLR(pin);
		PIN_HYS_SET(pin, enable);
		break;
	}
}

/*Inverts (or not) the input seen by a pin. */
void Chip_IOCON_PinSetInputInverted(LPC_IOCON_T *pIOCON, CHIP_PINx_T pin, bool invert)
{
	PIN_INV_CLR(pin);
	PIN_INV_SET(pin, invert);
}

/* Enables/disables Open-Drain mode for a pin. */
void Chip_IOCON_PinSetOpenDrainMode(LPC_IOCON_T *pIOCON, CHIP_PINx_T pin, bool open_drain)
{
	PIN_OD_CLR(pin);
	PIN_OD_SET(pin, open_drain);
}

/* Enable/configure digital filter sample mode for a pin. */
void Chip_IOCON_PinSetSampleMode(LPC_IOCON_T *pIOCON, CHIP_PINx_T pin, CHIP_PIN_SMODE_T smode)
{
	PIN_SMODE_CLR(pin);
	PIN_SMODE_SET(pin, smode);
}

/* Set the peripheral clock divisor for a pin. */
void Chip_IOCON_PinSetClockDivisor(LPC_IOCON_T *pIOCON, CHIP_PINx_T pin, CHIP_PIN_CLKDIV_T clkdiv)
{
	PIN_CLKDIV_CLR(pin);
	PIN_CLKDIV_SET(pin, clkdiv);
}

/* Set the I2C mode for a pin. */
void Chip_IOCON_PinSetI2CMode(LPC_IOCON_T *pIOCON, CHIP_PINx_T pin, CHIP_PIN_I2CMODE_T mode)
{
	switch (pin) {
	case PIO10:
	case PIO11:
		PIN_I2CMODE_CLR(pin);
		PIN_I2CMODE_SET(pin, mode);
		break;

	default:	/* Do nothing for non-I2C specific pins */
		break;
	}
}
