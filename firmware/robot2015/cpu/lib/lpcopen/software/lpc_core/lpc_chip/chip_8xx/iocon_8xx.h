/*
 * @brief LPC8xx IOCON register block and driver
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

#ifndef __IOCON_8XX_H_
#define __IOCON_8XX_H_

#ifdef __cplusplus
extern "C" {
#endif

/** @defgroup IOCON_8XX CHIP: LPC8xx IOCON register block and driver
 * @ingroup CHIP_8XX_Drivers
 * @{
 */

/**
 * @brief IOCON register block structure
 */
typedef struct {							/*!< (@ 0x40044000) IOCONFIG Structure     */
	__IO uint32_t PIO0_17;					/*!< (@ 0x40044000) I/O configuration for pin PIO0_17 */
	__IO uint32_t PIO0_13;					/*!< (@ 0x40044004) I/O configuration for pin PIO0_13 */
	__IO uint32_t PIO0_12;					/*!< (@ 0x40044008) I/O configuration for pin PIO0_12 */
	__IO uint32_t PIO0_5;					/*!< (@ 0x4004400C) I/O configuration for pin PIO0_5 */
	__IO uint32_t PIO0_4;					/*!< (@ 0x40044010) I/O configuration for pin PIO0_4 */
	__IO uint32_t PIO0_3;					/*!< (@ 0x40044014) I/O configuration for pin PIO0_3 */
	__IO uint32_t PIO0_2;					/*!< (@ 0x40044018) I/O configuration for pin PIO0_2 */
	__IO uint32_t PIO0_11;					/*!< (@ 0x4004401C) I/O configuration for pin PIO0_11 */
	__IO uint32_t PIO0_10;					/*!< (@ 0x40044020) I/O configuration for pin PIO0_10 */
	__IO uint32_t PIO0_16;					/*!< (@ 0x40044024) I/O configuration for pin PIO0_16 */
	__IO uint32_t PIO0_15;					/*!< (@ 0x40044028) I/O configuration for pin PIO0_15 */
	__IO uint32_t PIO0_1;					/*!< (@ 0x4004402C) I/O configuration for pin PIO0_1 */
	__IO uint32_t Reserved;					/*!< (@ 0x40044030) Reserved */
	__IO uint32_t PIO0_9;					/*!< (@ 0x40044034) I/O configuration for pin PIO0_9 */
	__IO uint32_t PIO0_8;					/*!< (@ 0x40044038) I/O configuration for pin PIO0_8 */
	__IO uint32_t PIO0_7;					/*!< (@ 0x4004403C) I/O configuration for pin PIO0_7 */
	__IO uint32_t PIO0_6;					/*!< (@ 0x40044040) I/O configuration for pin PIO0_6 */
	__IO uint32_t PIO0_0;					/*!< (@ 0x40044044) I/O configuration for pin PIO0_0 */
	__IO uint32_t PIO0_14;					/*!< (@ 0x40044048) I/O configuration for pin PIO0_14 */
} LPC_IOCON_T;

/**
 * @brief IOCON Pin Numbers enum
 */
typedef enum CHIP_PINx {
	PIO0  =  0,		/*!< PIN 0 */
	PIO1  =  1,		/*!< PIN 1 */
	PIO2  =  2,		/*!< PIN 2 */
	PIO3  =  3,		/*!< PIN 3 */
	PIO4  =  4,		/*!< PIN 4 */
	PIO5  =  5,		/*!< PIN 5 */
	PIO6  =  6,		/*!< PIN 6 */
	PIO7  =  7,		/*!< PIN 7 */
	PIO8  =  8,		/*!< PIN 8 */
	PIO9  =  9,		/*!< PIN 9 */
	PIO10 = 10,		/*!< PIN 10 */
	PIO11 = 11,		/*!< PIN 11 */
	PIO12 = 12,		/*!< PIN 12 */
	PIO13 = 13,		/*!< PIN 13 */
	PIO14 = 14,		/*!< PIN 14 */
	PIO15 = 15,		/*!< PIN 15 */
	PIO16 = 16,		/*!< PIN 16 */
	PIO17 = 17,		/*!< PIN 17 */
	PIO_NUL = 0xFF	/*!< PIN NULL */
} CHIP_PINx_T;

/**
 * @brief IOCON Pin Modes enum
 */
typedef enum CHIP_PIN_MODE {
	PIN_MODE_INACTIVE = 0,	/*!< Inactive mode */
	PIN_MODE_PULLDN = 1,	/*!< Pull Down mode */
	PIN_MODE_PULLUP = 2,	/*!< Pull up mode */
	PIN_MODE_REPEATER = 3	/*!< Repeater mode */
} CHIP_PIN_MODE_T;

/**
 * @brief IOCON Digital Filter Sample modes enum
 */
typedef enum CHIP_PIN_SMODE {
	PIN_SMODE_BYPASS = 0,	/*!< Bypass input filter */
	PIN_SMODE_CYC1 = 1,		/*!< Input pulses shorter than 1 filter clock cycle are rejected */
	PIN_SMODE_CYC2 = 2,		/*!< Input pulses shorter than 2 filter clock cycles are rejected */
	PIN_SMODE_CYC3 = 3		/*!< Input pulses shorter than 3 filter clock cycles are rejected */
} CHIP_PIN_SMODE_T;

/**
 * @brief IOCON Perpipheral Clock divider selction for input filter
 * sampling clock
 */
typedef enum CHIP_PIN_CLKDIV {
	IOCONCLKDIV0 = 0,	/*!< Clock divider 0 */
	IOCONCLKDIV1 = 1,	/*!< Clock divider 1 */
	IOCONCLKDIV2 = 2,	/*!< Clock divider 2 */
	IOCONCLKDIV3 = 3,	/*!< Clock divider 3 */
	IOCONCLKDIV4 = 4,	/*!< Clock divider 4 */
	IOCONCLKDIV5 = 5,	/*!< Clock divider 5 */
	IOCONCLKDIV6 = 6	/*!< Clock divider 6 */
} CHIP_PIN_CLKDIV_T;

/**
 * @brief IOCON I2C Modes enum (Only for I2C pins PIO0_10 and PIO0_11)
 */
typedef enum CHIP_PIN_I2CMODE {
	PIN_I2CMODE_STDFAST = 0,	/*!< I2C standard mode/Fast mode */
	PIN_I2CMODE_GPIO = 1,		/*!< Standard I/O functionality */
	PIN_I2CMODE_FASTPLUS = 2	/*!< I2C Fast plus mode */
} CHIP_PIN_I2CMODE_T;

/**
 * @brief	Set a function mode (pull-up/pull-down) for a pin
 * @param	pIOCON	: The base of IOCON peripheral on the chip
 * @param	pin		: Pin number
 * @param	mode	 Mode (Pull-up/Pull-down mode)
 * @return	Nothing
 */
void Chip_IOCON_PinSetMode(LPC_IOCON_T *pIOCON, CHIP_PINx_T pin, CHIP_PIN_MODE_T mode);

/**
 * @brief	Enable or disable the hysteresis for a pin
 * @param	pIOCON	: The base of IOCON peripheral on the chip
 * @param	pin		: Pin number
 * @param	enable	: true to enable, false to disable
 * @return	Nothing
 */
void Chip_IOCON_PinSetHysteresis(LPC_IOCON_T *pIOCON, CHIP_PINx_T pin, bool enable);

/**
 * @brief	Invert input for a pin
 * @param	pIOCON	: The base of IOCON peripheral on the chip
 * @param	pin		: Pin number
 * @param	invert	: true to invert, false to not to invert
 * @return	Nothing
 */
void Chip_IOCON_PinSetInputInverted(LPC_IOCON_T *pIOCON, CHIP_PINx_T pin, bool invert);

/**
 * @brief	Sets open-drain mode for a pin
 * @param	pIOCON		: The base of IOCON peripheral on the chip
 * @param	pin			: Pin number
 * @param	open_drain	: true to enable open-drain mode,
 *                        false to disable open-drain mode
 * @return	Nothing
 */
void Chip_IOCON_PinSetOpenDrainMode(LPC_IOCON_T *pIOCON, CHIP_PINx_T pin, bool open_drain);

/**
 * @brief	Sets the digital filter sampling mode for a pin
 * @param	pIOCON	: The base of IOCON peripheral on the chip
 * @param	pin		: Pin number
 * @param	smode	: 0x0 = bypass, 0x[1..3] = 1 to 3 clock cycles.
 * @return	Nothing
 */
void Chip_IOCON_PinSetSampleMode(LPC_IOCON_T *pIOCON, CHIP_PINx_T pin, CHIP_PIN_SMODE_T smode);

/**
 * @brief	Select peripheral clock divider for input filter sampling clock
 * @param	pIOCON	: The base of IOCON peripheral on the chip
 * @param	pin		: Pin number
 * @param	clkdiv	: 0 = no divisor, 1...6 = PCLK/clkdiv
 * @return	Nothing
 */
void Chip_IOCON_PinSetClockDivisor(LPC_IOCON_T *pIOCON, CHIP_PINx_T pin, CHIP_PIN_CLKDIV_T clkdiv);

/**
 * @brief	Set I2C mode for a pin
 * @param	pIOCON	: The base of IOCON peripheral on the chip
 * @param	pin		: Pin number
 * @param	mode	: 0:Standard/Fast I2C 1: GPIO 2: Fast Plus
 * @return	Nothing
 * @note	Valid for pins PIO0_10 and PIO0_11
 */
void Chip_IOCON_PinSetI2CMode(LPC_IOCON_T *pIOCON, CHIP_PINx_T pin, CHIP_PIN_I2CMODE_T mode);

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* __IOCON_8XX_H_ */
