/*
 * @brief LPC17xx/40xx UART chip driver
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

/** UART Ring buffer declaration*/
static UART_RingBuffer_T rb;

/** Current Tx Interrupt enable state */
static __IO FlagStatus TxIntStat;

FlagStatus ABsyncSts = RESET;

/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/

/*****************************************************************************
 * Private functions
 ****************************************************************************/

/* Get UART number based on selected UART */
static IP_UART_ID_T Chip_UART_Get_UARTNum(LPC_USART_T *pUART)
{
	if (pUART == LPC_UART0) {
		return UART_0;
	}
	else if (pUART == LPC_UART1) {
		return UART_1;
	}
	else if (pUART == LPC_UART2) {
		return UART_2;
	}
	else if (pUART == LPC_UART3) {
		return UART_3;
	}

	return UART_4;
}

/* Determine UART clock based in selected UART */
static CHIP_SYSCTL_CLOCK_T Chip_UART_DetermineClk(LPC_USART_T *pUART) {
	CHIP_SYSCTL_CLOCK_T uartclk;

	/* Pick clock for uart BASED ON SELECTED uart */
	if (pUART == LPC_UART1) {
		uartclk = SYSCTL_CLOCK_UART1;
	}
	else if (pUART == LPC_UART2) {
		uartclk = SYSCTL_CLOCK_UART2;
	}
	else if (pUART == LPC_UART3) {
		uartclk = SYSCTL_CLOCK_UART3;
	}
#if !defined(CHIP_LPC175X_6X)
	else if (pUART == LPC_UART4) {
		uartclk = SYSCTL_CLOCK_UART4;
	}
#endif
	else {
		uartclk = SYSCTL_CLOCK_UART0;
	}
	return uartclk;
}

static uint32_t Chip_UART_GetClockRate(LPC_USART_T *pUART) {
#if !defined(CHIP_LPC175X_6X)
	return Chip_Clock_GetPeripheralClockRate();
#else
	/* Pick clock for uart BASED ON SELECTED uart */
	if (pUART == LPC_UART1) {
		return Chip_Clock_GetPeripheralClockRate(SYSCTL_PCLK_UART1);
	}
	else if (pUART == LPC_UART2) {
		return Chip_Clock_GetPeripheralClockRate(SYSCTL_PCLK_UART2);
	}
	else if (pUART == LPC_UART3) {
		return Chip_Clock_GetPeripheralClockRate(SYSCTL_PCLK_UART3);
	}
	else {
		return Chip_Clock_GetPeripheralClockRate(SYSCTL_PCLK_UART0);
	}
#endif
}

/*****************************************************************************
 * Public functions
 ****************************************************************************/

/* Initializes the pUART peripheral */
void Chip_UART_Init(LPC_USART_T *pUART)
{
	IP_UART_ID_T UARTPort = Chip_UART_Get_UARTNum(pUART);

	/* Enable UART clocking. UART base clock(s) must already be enabled */
	Chip_Clock_EnablePeriphClock(Chip_UART_DetermineClk(pUART));

	IP_UART_Init(pUART, UARTPort);
}

/* De-initializes the pUART peripheral */
void Chip_UART_DeInit(LPC_USART_T *pUART)
{
	IP_UART_ID_T UARTPort = Chip_UART_Get_UARTNum(pUART);

	IP_UART_DeInit(pUART, UARTPort);

	/* Disable UART clocking */
	Chip_Clock_DisablePeriphClock(Chip_UART_DetermineClk(pUART));
}

/* Determines best dividers to get a target baud rate */
Status Chip_UART_SetBaud(LPC_USART_T *pUART, uint32_t baudrate)
{
	uint32_t uClk;

	/* Get UART clock rate */
	uClk = Chip_UART_GetClockRate(pUART);

	return IP_UART_SetBaud(pUART, baudrate, uClk);
}

/* Enable/Disable transmission on UART TxD pin */
void Chip_UART_TxCmd(LPC_USART_T *pUART, FunctionalState NewState)
{
	IP_UART_ID_T UARTPort = Chip_UART_Get_UARTNum(pUART);

	IP_UART_TxCmd(pUART, UARTPort, NewState);
}

/* Get Interrupt Stream Status */
IP_UART_INT_STATUS_T Chip_UART_GetIntStatus(LPC_USART_T *pUART)
{
	uint32_t intsrc, tmp, tmp1;
	IP_UART_INT_STATUS_T ret = UART_INTSTS_ERROR;

	/* Determine the interrupt source */
	intsrc = Chip_UART_IntGetStatus(pUART);

	tmp = intsrc & UART_IIR_INTID_MASK;

	/* Receive Line Status */
	if (tmp == UART_IIR_INTID_RLS) {
		/* Check line status */
		tmp1 = (uint32_t) Chip_UART_GetLineStatus(pUART);
		/* Mask out the Receive Ready and Transmit Holding empty status */
		tmp1 &= (UART_LSR_OE | UART_LSR_PE | UART_LSR_FE \
				 | UART_LSR_BI | UART_LSR_RXFE);
		/* If any error exist */
		if (tmp1) {
			return UART_INTSTS_ERROR;
		}
	}

	/* Receive Data Available or Character time-out */
	if ((tmp == UART_IIR_INTID_RDA) || (tmp == UART_IIR_INTID_CTI)) {
		ret |= UART_INTSTS_RTR;
	}

	/* Transmit Holding Empty */
	if (tmp == UART_IIR_INTID_THRE) {
		ret |= UART_INTSTS_RTS;
	}

	if (intsrc & UART_IIR_ABEO_INT) {
		ret |= UART_INTSTS_ABEO;
	}
	else if (intsrc & UART_IIR_ABTO_INT) {
		ret |= UART_INTSTS_ABTO;
	}
	return ret;
}

/* UART interrupt service routine */
void Chip_UART_Interrupt_Handler(LPC_USART_T *pUART)
{
	uint8_t tmpc;
	uint32_t rLen;
	IP_UART_INT_STATUS_T Sts = Chip_UART_GetIntStatus(pUART);
	if (Sts == UART_INTSTS_ERROR) {
		return;	/* error */
	}
	if (Sts & UART_INTSTS_RTR) {	/* ready for Read Data */
		while (1) {
			/* Call UART read function in UART driver */
			rLen = Chip_UART_Receive(pUART, &tmpc, 1, NONE_BLOCKING);
			/* If data received */
			if (rLen) {
				/* Check if buffer is more space
				 * If no more space, remaining character will be trimmed out
				 */
				if (!__BUF_IS_FULL(rb.rx_head, rb.rx_tail)) {
					rb.rx[rb.rx_head] = tmpc;
					__BUF_INCR(rb.rx_head);
				}
			}
			/* no more data */
			else {
				break;
			}
		}
	}

	if (Sts & UART_INTSTS_RTS) {	/* ready for Write Data */
		/* Disable THRE interrupt */
		Chip_UART_IntConfig(pUART, UART_INTCFG_THRE, DISABLE);

		/* Wait for FIFO buffer empty, transfer UART_TX_FIFO_SIZE bytes
		 * of data or break whenever ring buffers are empty */
		/* Wait until THR empty */
		while (Chip_UART_CheckBusy(pUART) == SET) {}

		while (!__BUF_IS_EMPTY(rb.tx_head, rb.tx_tail)) {
			/* Move a piece of data into the transmit FIFO */
			if (Chip_UART_Send(pUART, (uint8_t *) &rb.tx[rb.tx_tail], 1, NONE_BLOCKING)) {
				/* Update transmit ring FIFO tail pointer */
				__BUF_INCR(rb.tx_tail);
			}
			else {
				break;
			}
		}

		/* If there is no more data to send, disable the transmit
		   interrupt - else enable it or keep it enabled */
		if (__BUF_IS_EMPTY(rb.tx_head, rb.tx_tail)) {
			Chip_UART_IntConfig(pUART, UART_INTCFG_THRE, DISABLE);
			// Reset Tx Interrupt state
			TxIntStat = RESET;
		}
		else {
			/* Set Tx Interrupt state */
			TxIntStat = SET;
			Chip_UART_IntConfig(pUART, UART_INTCFG_THRE, ENABLE);
		}
	}

	if (Sts & UART_INTSTS_ABEO) {
		Chip_UART_ABClearIntPending(pUART, UART_INTSTS_ABEO);
	}
	if (Sts & UART_INTSTS_ABTO) {
		Chip_UART_ABClearIntPending(pUART, UART_INTSTS_ABTO);
	}
	if (ABsyncSts == RESET) {
		/* Interrupt caused by End of auto-baud */
		if (Sts & UART_INTSTS_ABEO) {
			// Disable AB interrupt
			Chip_UART_IntConfig(pUART, UART_INTCFG_ABEO, DISABLE);
			// Set Sync flag
			ABsyncSts = SET;
		}

		/* Auto-Baudrate Time-Out interrupt (not implemented) */
		if (Sts & UART_INTSTS_ABTO) {
			/* Disable this interrupt - Add your code here */
			Chip_UART_IntConfig(pUART, UART_INTCFG_ABTO, DISABLE);
		}
	}
}

/* UART transmit function for interrupt mode (using ring buffers) */
uint32_t Chip_UART_Interrupt_Transmit(LPC_USART_T *pUART, uint8_t *txbuf, uint8_t buflen)
{
	uint8_t *data = (uint8_t *) txbuf;
	uint32_t bytes = 0;

	/* Temporarily lock out UART transmit interrupts during this
	   read so the UART transmit interrupt won't cause problems
	   with the index values */
	Chip_UART_IntConfig(pUART, UART_INTCFG_THRE, DISABLE);

	/* Loop until transmit run buffer is full or until n_bytes
	   expires */
	while ((buflen > 0) && (!__BUF_IS_FULL(rb.tx_head, rb.tx_tail))) {
		/* Write data from buffer into ring buffer */
		rb.tx[rb.tx_head] = *data;
		data++;

		/* Increment head pointer */
		__BUF_INCR(rb.tx_head);

		/* Increment data count and decrement buffer size count */
		bytes++;
		buflen--;
	}

	/*
	 * Check if current Tx interrupt enable is reset,
	 * that means the Tx interrupt must be re-enabled
	 * due to call UART_IntTransmit() function to trigger
	 * this interrupt type
	 */
	if (TxIntStat == RESET) {
		// Disable THRE interrupt
		Chip_UART_IntConfig(pUART, UART_INTCFG_THRE, DISABLE);

		/* Wait for FIFO buffer empty, transfer UART_TX_FIFO_SIZE bytes
		 * of data or break whenever ring buffers are empty */
		/* Wait until THR empty */
		while (Chip_UART_CheckBusy(pUART) == SET) {}

		while (!__BUF_IS_EMPTY(rb.tx_head, rb.tx_tail)) {
			/* Move a piece of data into the transmit FIFO */
			if (Chip_UART_Send(pUART, (uint8_t *) &rb.tx[rb.tx_tail], 1, NONE_BLOCKING)) {
				/* Update transmit ring FIFO tail pointer */
				__BUF_INCR(rb.tx_tail);
			}
			else {
				break;
			}
		}

		/* If there is no more data to send, disable the transmit
		   interrupt - else enable it or keep it enabled */
		if (__BUF_IS_EMPTY(rb.tx_head, rb.tx_tail)) {
			Chip_UART_IntConfig(pUART, UART_INTCFG_THRE, DISABLE);
			/* Reset Tx Interrupt state */
			TxIntStat = RESET;
		}
		else {
			/* Set Tx Interrupt state */
			TxIntStat = SET;
			Chip_UART_IntConfig(pUART, UART_INTCFG_THRE, ENABLE);
		}
	}
	/*
	 * Otherwise, re-enables Tx Interrupt
	 */
	else {
		Chip_UART_IntConfig(pUART, UART_INTCFG_THRE, ENABLE);
	}

	return bytes;
}

/* UART read function for interrupt mode (using ring buffers) */
uint32_t Chip_UART_Interrupt_Receive(LPC_USART_T *pUART, uint8_t *rxbuf, uint8_t buflen)
{
	uint8_t *data = (uint8_t *) rxbuf;
	uint32_t bytes = 0;

	/* Temporarily lock out UART receive interrupts during this
	   read so the UART receive interrupt won't cause problems
	   with the index values */
	Chip_UART_IntConfig(pUART, UART_INTCFG_RBR, DISABLE);

	/* Loop until receive buffer ring is empty or
	    until max_bytes expires */
	while ((buflen > 0) && (!(__BUF_IS_EMPTY(rb.rx_head, rb.rx_tail)))) {
		/* Read data from ring buffer into user buffer */
		*data = rb.rx[rb.rx_tail];
		data++;

		/* Update tail pointer */
		__BUF_INCR(rb.rx_tail);

		/* Increment data count and decrement buffer size count */
		bytes++;
		buflen--;
	}

	/* Re-enable UART interrupts */
	Chip_UART_IntConfig(pUART, UART_INTCFG_RBR, ENABLE);

	return bytes;
}

/* Reset Tx and Rx ring buffer (head and tail) */
void Chip_UART_InitRingBuffer(LPC_USART_T *pUART)
{
	(void) pUART;

	TxIntStat = RESET;

	/* Reset ring buf head and tail idx */
	__BUF_RESET(rb.rx_head);
	__BUF_RESET(rb.rx_tail);
	__BUF_RESET(rb.tx_head);
	__BUF_RESET(rb.tx_tail);
}

/* UART interrupt service routine */
FlagStatus Chip_UART_GetABEOStatus(LPC_USART_T *pUART)
{
	(void) pUART;

	return ABsyncSts;
}
