/*
 * @brief	UART/USART registers and control functions
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

#include "usart_004.h"

/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/

/* Baud-rate pre-scaler multiplier value */
#define UART_FDR_MULVAL(n) (((n) << 4) & 0xF0)

/* Acceptable UART baudrate error */
#define UART_ACCEPTED_BAUDRATE_ERROR (3)

/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/

/*****************************************************************************
 * Private functions
 ****************************************************************************/

/*****************************************************************************
 * Public functions
 ****************************************************************************/

/* Basic UART initialization */
void IP_UART_Init(IP_USART_001_T *pUART)
{
	/* Enable FIFOs by default, reset them */
	IP_UART_SetupFIFOS(pUART, (UART_FCR_FIFO_EN | UART_FCR_RX_RS | UART_FCR_TX_RS));

	/* Default 8N1, with DLAB disabled */
	IP_UART_SetMode(pUART, (UART_LCR_WLEN8 | UART_LCR_SBS_1BIT | UART_LCR_PARITY_DIS));

	/* Disable fractional divider */
	pUART->FDR = 0x10;
}

// FIXME - this function doesn't work correctly and is too big
/* Determines and sets best dividers to get a target baud rate */
uint32_t IP_UART_SetBaud(IP_USART_001_T *pUART, uint32_t baudrate, uint32_t uClk)
{
	uint32_t actualRate = 0, d, m, bestd, bestm, tmp;
	uint64_t best_divisor, divisor;
	uint32_t current_error, best_error;
	uint32_t recalcbaud;

	/* In the Uart IP block, baud rate is calculated using FDR and DLL-DLM registers
	 * The formula is :
	 * BaudRate= uClk * (mulFracDiv/(mulFracDiv+dividerAddFracDiv) / (16 * (DLL)
	 * It involves floating point calculations. That's the reason the formulae are adjusted with
	 * Multiply and divide method.*/
	/* The value of mulFracDiv and dividerAddFracDiv should comply to the following expressions:
	 * 0 < mulFracDiv <= 15, 0 <= dividerAddFracDiv <= 15 */
	best_error = 0xFFFFFFFF;/* Worst case */
	bestd = 0;
	bestm = 0;
	best_divisor = 0;
	for (m = 1; m <= 15; m++) {
		for (d = 0; d < m; d++) {
			divisor = ((uint64_t) uClk << 28) * m / (baudrate * (m + d));
			current_error = divisor & 0xFFFFFFFF;

			tmp = divisor >> 32;

			/* Adjust error */
			if (current_error > ((uint32_t) 1 << 31)) {
				current_error = -current_error;
				tmp++;
			}

			if (( tmp < 1) || ( tmp > 65536)) {
				/* Out of range */
				continue;
			}

			if ( current_error < best_error) {
				best_error = current_error;
				best_divisor = tmp;
				bestd = d;
				bestm = m;
				if (best_error == 0) {
					break;
				}
			}
		}	/* for (d) */

		if (best_error == 0) {
			break;
		}
	}	/* for (m) */

	if (best_divisor == 0) {
		/* can not find best match */
		return 0;
	}

	recalcbaud = (uClk >> 4) * bestm / (best_divisor * (bestm + bestd));

	/* reuse best_error to evaluate baud error */
	if (baudrate > recalcbaud) {
		best_error = baudrate - recalcbaud;
	}
	else {
		best_error = recalcbaud - baudrate;
	}

	best_error = (best_error * 100) / baudrate;

	if (best_error < UART_ACCEPTED_BAUDRATE_ERROR) {
		IP_UART_EnableDivisorAccess(pUART);
		// FIXME - this is not how DLL and DLM work - they do not add
		IP_UART_SetDivisorLatches(pUART, best_divisor, best_divisor);
		IP_UART_DisableDivisorAccess(pUART);

		/* Set best fractional divider */
		pUART->FDR = (UART_FDR_MULVAL(bestm) | bestd);

		/* Return actual baud rate */
		actualRate = recalcbaud;
	}

	return actualRate;
}
