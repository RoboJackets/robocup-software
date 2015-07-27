/*
 * @brief LPC8xx UART driver
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

/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/

/*****************************************************************************
 * Private functions
 ****************************************************************************/

/*****************************************************************************
 * Public functions
 ****************************************************************************/

/* Initialize the UART peripheral */
void Chip_UART_Init(LPC_USART_T *pUART)
{
	/* Clear the interrupt */
	IP_UART_ClearStatus(pUART, CTS_DELTA | DELTA_RXBRK);

	/* Enable the UART */
	IP_UART_Init(pUART);
}

/* Set baud rate for UART */
void Chip_UART_SetBaudRate(LPC_USART_T *pUART, uint32_t baudrate)
{
	uint32_t err, uart_fra_multiplier, baudRateGenerator;
	uint32_t systemCoreClock = Chip_Clock_GetMainClockRate();

	/* Calculate baudrate generator value */
	baudRateGenerator = systemCoreClock / (16 * baudrate);
	err = systemCoreClock - baudRateGenerator * 16 * baudrate;
	uart_fra_multiplier = (err * 0xFF) / (baudRateGenerator * 16 * baudrate);
	IP_UART_SetBaudRate(pUART, baudRateGenerator - 1);		/*baud rate */
	Chip_SYSCTL_SetUSARTFRGDivider(0xFF);	/* value 0xFF is always used */
	Chip_SYSCTL_SetUSARTFRGMultiplier(uart_fra_multiplier);
}

/* Transmit a 8 bits data */
Status Chip_UART_SendByte(LPC_USART_T *pUART, uint8_t data)
{
	if (IP_UART_GetStatus(pUART) & (TXRDY)) {
		IP_UART_Transmit(pUART, data);
		return SUCCESS;
	}
	return ERROR;
}

/* Receive a 8 bits data */
Status Chip_UART_ReceiveByte(LPC_USART_T *pUART, uint8_t *Data)
{
	if (IP_UART_GetStatus(pUART) & (RXRDY)) {
		*Data = IP_UART_Receive(pUART);
		return SUCCESS;
	}
	return ERROR;
}
