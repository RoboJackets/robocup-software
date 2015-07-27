/*
 * @brief NXP XPresso 812 board file
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

#ifndef __BOARD_NXP_XPRESSO_812_H_
#define __BOARD_NXP_XPRESSO_812_H_

#include "chip.h"
#include "board_api.h"

#if defined(DEBUG_ENABLE) && defined(DEBUGOUT)
#undef DEBUGOUT
/* Normally routed to printf, but uses too much space, so stub it */
#define DEBUGOUT(...)
#endif

#ifdef __cplusplus
extern "C" {
#endif

/** @defgroup BOARD_NXP_XPRESSO_812 NXP LPC812 Xpresso board support functions
 * @ingroup BOARDS_8XX
 * @{
 */

/** @defgroup BOARD_NXP_XPRESSO_812_OPTIONS BOARD: LPC812 board build options
 * This board has options that configure its operation at build-time.<br>
 *
 * For more information on driver options see<br>
 * @ref LPCOPEN_DESIGN_ARPPROACH<br>
 * @{
 */

#ifndef LPC8XX_USE_XTAL_OSC
#define LPC8XX_USE_XTAL_OSC         1 /* Set 0 if using on-chip IRC instead of external oscillator */
#endif

/**
 * @}
 */

#define BOARD_NXP_XPRESSO_812

/* ACMP Configuration */
#define LPC8XX_USE_ACMP_I1           (1)    /* 0 to not configure ACMP_I1 to use pin choice below */
#define LPC8XX_ACMP_O_PINx          (PIO15)	/* This value selects the output P0.x pin for the ACMP Output Function */

/**
 * @brief	Initialize pin muxing for UART interface
 * @param	pUART	: Pointer to SPI interface to initialize
 * @return	Nothing
 */
void Board_UART_Init(LPC_USART_T *pUART);

/**
 * @brief	Initialize pin muxing for SPI interface
 * @param	pSPI	: Pointer to SPI interface to initialize
 * @return	Nothing
 */
void Board_SPI_Init(LPC_SPI_T *pSPI);

/**
 * @brief	Initialize pin muxing and clock for I2C interface
 * @return	Nothing
 */
void Board_I2C_Init(void);
/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* __BOARD_NXP_XPRESSO_812_H_ */
