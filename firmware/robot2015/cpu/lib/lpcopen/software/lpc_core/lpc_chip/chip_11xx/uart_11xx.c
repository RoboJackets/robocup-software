/*
 * @brief LPC11xx UART chip driver
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

#include "chip.h"

/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/

/* Used for transmit interrupt monitoring */
STATIC bool txIntEnabled;

/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/

/*****************************************************************************
 * Private functions
 ****************************************************************************/

/*****************************************************************************
 * Public functions
 ****************************************************************************/

/* Initializes the pUART peripheral */
void Chip_UART_Init(LPC_USART_T *pUART)
{
	/* Enable clock to UART block */
	Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_UART0);
	Chip_Clock_SetUARTClockDiv(1);

	IP_UART_Init(pUART);
}

/* De-initializes the pUART peripheral */
void Chip_UART_DeInit(LPC_USART_T *pUART)
{
	IP_UART_DeInit(pUART);

	/* Disable UART block */
	Chip_Clock_DisablePeriphClock(SYSCTL_CLOCK_UART0);
}

/* Transmit a byte array through the UART peripheral (non-blocking) */
int Chip_UART_Send(LPC_USART_T *pUART, const void *data, int numBytes)
{
	int sent = 0;
	uint8_t *p8 = (uint8_t *) data;

	/* Send until the transmit FIFO is full or out of bytes */
	while ((sent < numBytes) &&
		   ((IP_UART_ReadLineStatus(pUART) & UART_LSR_THRE) != 0)) {
		IP_UART_SendByte(pUART, *p8);
		p8++;
		sent++;
	}

	return sent;
}

/* Transmit a byte array through the UART peripheral (blocking) */
int Chip_UART_SendBlocking(LPC_USART_T *pUART, const void *data, int numBytes)
{
	int pass, sent = 0;
	uint8_t *p8 = (uint8_t *) data;

	while (numBytes > 0) {
		pass = Chip_UART_Send(pUART, p8, numBytes);
		numBytes -= pass;
		sent += pass;
		p8 += pass;
	}

	return sent;
}

/* Read data through the UART peripheral (non-blocking) */
int Chip_UART_Read(LPC_USART_T *pUART, void *data, int numBytes)
{
	int readBytes = 0;
	uint8_t *p8 = (uint8_t *) data;

	/* Send until the transmit FIFO is full or out of bytes */
	while ((readBytes < numBytes) &&
		   ((IP_UART_ReadLineStatus(pUART) & UART_LSR_RDR) != 0)) {
		*p8 = IP_UART_ReadByte(pUART);
		p8++;
		readBytes++;
	}

	return readBytes;
}

/* Read data through the UART peripheral (blocking) */
int Chip_UART_ReadBlocking(LPC_USART_T *pUART, void *data, int numBytes)
{
	int pass, readBytes = 0;
	uint8_t *p8 = (uint8_t *) data;

	while (readBytes < numBytes) {
		pass = Chip_UART_Read(pUART, p8, numBytes);
		numBytes -= pass;
		readBytes += pass;
		p8 += pass;
	}

	return readBytes;
}

/* Determines and sets best dividers to get a target bit rate */
uint32_t Chip_UART_SetBaud(LPC_USART_T *pUART, uint32_t baudrate)
{
	uint32_t div, divh, divl, clkin;

	/* Determine UART clock in rate without FDR */
	clkin = Chip_Clock_GetMainClockRate();
	div = clkin / (baudrate * 16);

	/* High and low halves of the divider */
	divh = div / 256;
	divl = div - (divh * 256);

	IP_UART_EnableDivisorAccess(pUART);
	IP_UART_SetDivisorLatches(pUART, divl, divh);
	IP_UART_DisableDivisorAccess(pUART);

	/* Fractional FDR alreadt setup for 1 in UART init */

	return clkin / div;
}

/* UART receive-only interrupt handler for ring buffers */
void Chip_UART_RXIntHandlerRB(LPC_USART_T *pUART, RINGBUFF_T *pRB)
{
	/* New data will be ignored if data not popped in time */
	while (Chip_UART_ReadLineStatus(pUART) & UART_LSR_RDR) {
		uint8_t ch = Chip_UART_ReadByte(pUART);
		RingBuffer_Insert(pRB, &ch);
	}
}

/* UART transmit-only interrupt handler for ring buffers */
void Chip_UART_TXIntHandlerRB(LPC_USART_T *pUART, RINGBUFF_T *pRB)
{
	uint8_t ch;

	/* Fill FIFO until full or until TX ring buffer is empty */
	while ((IP_UART_ReadLineStatus(pUART) & UART_LSR_THRE) != 0 &&
		   RingBuffer_Pop(pRB, &ch)) {
		IP_UART_SendByte(pUART, ch);
	}

	/* Turn off interrupt if the ring buffer is empty */
	if (RingBuffer_IsEmpty(pRB))
	{
		/* Shut down transmit */
		Chip_UART_IntDisable(pUART, UART_IER_THREINT);
		txIntEnabled = false;
	}
}

/* Populate a transmit ring buffer and start UART transmit */
uint32_t Chip_UART_SendRB(LPC_USART_T *pUART, RINGBUFF_T *pRB, const void *data, int bytes)
{
	int ret;
	uint8_t *p8 = (uint8_t *) data;

	/* Fill transmit FIFO */
	txIntEnabled = false;
	ret = RingBuffer_InsertMult(pRB, p8, bytes);
	bytes -= ret;
	p8 += ret;
	Chip_UART_TXIntHandlerRB(pUART, pRB);
	txIntEnabled = true;

	/* Enable Transmit FIFO interrupt to start transmit ring
	   buffer servicing */
	Chip_UART_IntEnable(pUART, UART_IER_THREINT);

	/* Do this till all bytes are queued */
	while (bytes) {
		/* A proper wait handler must be added here */
		ret = RingBuffer_InsertMult(pRB, p8, bytes);
		bytes -= ret;
		p8 += ret;
	}

	return 0;
}

/* Copy data from a receive ring buffer */
int Chip_UART_ReadRB(LPC_USART_T *pUART, RINGBUFF_T *pRB, void *data, int bytes)
{
	(void) pUART;

	return RingBuffer_PopMult(pRB, (uint8_t *) data, bytes);
}

/* UART receive/transmit interrupt handler for ring buffers */
void Chip_UART_IRQRBHandler(LPC_USART_T *pUART, RINGBUFF_T *pRXRB, RINGBUFF_T *pTXRB)
{
	(void) pUART;

	/* Handle transmit and receive interrupts using the ring
	   buffer handlers */
	Chip_UART_RXIntHandlerRB(pUART, pRXRB);

	if (txIntEnabled) {
		Chip_UART_TXIntHandlerRB(pUART, pTXRB);
	}
}
