/*
 * @brief NXP XPRESSO 11C24 board file
 *
 * @note
 * Copyright(C) NXP Semiconductors, 2012
 * All rights reserved.
 *
 * @par
 * Software that is described herein is for illustrative purposes only
 * which provides customers with programming information regarding the
 * LPC products.  This software is supplied "AS IS" without any warranties of
 * any kind, and NXP Semiconductors and its licensor disclaim any and
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

#ifndef __BOARD_NXP_XPRESSO_11C24_H_
#define __BOARD_NXP_XPRESSO_11C24_H_

#include "chip.h"
#include "board_api.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @defgroup BOARD_NXP_XPRESSO_11C24 NXP LPC11C24 Xpresso board support functions
 * @ingroup BOARDS_11XX
 * @{
 */

/** @defgroup BOARD_NXP_XPRESSO_11C24_OPTIONS BOARD: NXP LPC11C24 Xpresso board build options
 * This board has options that configure its operation at build-time.<br>
 *
 * For more information on driver options see<br>
 * @ref LPCOPEN_DESIGN_ARPPROACH<br>
 */

/**
 * @}
 */

#define BOARD_NXP_XPRESSO_11C24

#if defined(DEBUG_ENABLE) && defined(DEBUGOUT)
#undef DEBUGOUT
/* Normally routed to printf, but uses too much space, so stub it */
#define DEBUGOUT(...)
#endif

/**
 * @brief	Initialize pin muxing for UART interface
 * @return	Nothing
 */
void Board_UART_Init(void);

/**
 * @brief	Initialize pin muxing for SSP interface
 * @return	Nothing
 */
void Board_SSP_Init(void);

/**
 * @brief	Initialize pin muxing for ADC channel 0
 * @return	Nothing
 */
void Board_ADC_Init(void);
/**
 * @brief	Initialize I2C Pins for I2C Operation
 * @param	id	: Must be passed as I2C0
 * @return	Nothing
 */
STATIC INLINE void Board_I2C_Init(I2C_ID_T id)
{
	Chip_SYSCTL_PeriphReset(RESET_I2C0);
	Chip_IOCON_PinMuxSet(LPC_IOCON, IOCON_PIO0_4, IOCON_FUNC1);
	Chip_IOCON_PinMuxSet(LPC_IOCON, IOCON_PIO0_5, IOCON_FUNC1);
}

/**
 * @brief	Sets up I2C Fast Plus Mode
 * @param	id	: Must always be I2C0
 * @return	Nothing
 * @note	For proper operation of I2C in Fast plus mode,
 *      	i.e, frequency above 400 KHz to 1 MHz,
 *      	this function must be called before setting up
 *      	I2C clock using Chip_I2C_SetClockRate()
 */
STATIC INLINE void Board_I2C_EnableFastPlus(I2C_ID_T id)
{
	Chip_IOCON_PinMuxSet(LPC_IOCON, IOCON_PIO0_4, IOCON_FUNC1 | IOCON_FASTI2C_EN);
	Chip_IOCON_PinMuxSet(LPC_IOCON, IOCON_PIO0_5, IOCON_FUNC1 | IOCON_FASTI2C_EN);
}

/**
 * @brief	Disables I2C Fast plus mode and enables normal mode
 * @param	id	: Must always be I2C0
 * @return	Nothing
 */
STATIC INLINE void Board_I2C_DisableFastPlus(I2C_ID_T id)
{
	Chip_IOCON_PinMuxSet(LPC_IOCON, IOCON_PIO0_4, IOCON_FUNC1);
	Chip_IOCON_PinMuxSet(LPC_IOCON, IOCON_PIO0_5, IOCON_FUNC1);
}

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* __BOARD_NXP_XPRESSO_11C24_H_ */
