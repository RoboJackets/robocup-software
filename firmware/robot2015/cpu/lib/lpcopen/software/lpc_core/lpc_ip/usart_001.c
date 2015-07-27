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

#include "usart_001.h"

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

/* Initializes the UARTx peripheral according to the specified parameters in
   the UART_ConfigStruct */
void IP_UART_Init(IP_USART_001_T *pUART, IP_UART_ID_T UARTPort)
{
	volatile uint32_t tmp;
	(void) tmp;	/* For use warnings only */

	/* FIFOs are empty */
	pUART->FCR = ( UART_FCR_FIFO_EN | UART_FCR_RX_RS | UART_FCR_TX_RS);
	/* Disable FIFO */
	// pUART->FCR = 0;
	/* Dummy reading */
	while (pUART->LSR & UART_LSR_RDR) {
		tmp = pUART->RBR;
	}
#if defined(CHIP_LPC177X_8X) || defined(CHIP_LPC407X_8X) || defined(CHIP_LPC18XX) || defined(CHIP_LPC43XX)
	switch (UARTPort) {
#if defined(CHIP_LPC177X_8X) || defined(CHIP_LPC407X_8X)
	case 4:
#else
	case 0:
	case 2:
	case 3:
#endif
		pUART->TER2 = UART_TER2_TXEN;
		/* Wait for current transmit complete */
		while (!(pUART->LSR & UART_LSR_THRE)) {}
		/* Disable Tx */
		pUART->TER2 = 0;
		break;

	default:
		break;
	}
#else
	pUART->TER1 = UART_TER1_TXEN;
	/* Wait for current transmit complete */
	while (!(pUART->LSR & UART_LSR_THRE)) {}
	/* Disable Tx */
	pUART->TER1 = 0;
#endif
	/* Disable interrupt */
	pUART->IER = 0;
	/* Set LCR to default state */
	pUART->LCR = 0;
	/* Set ACR to default state */
	pUART->ACR = 0;
#if defined(CHIP_LPC175X_6X) || defined(CHIP_LPC177X_8X) || defined(CHIP_LPC407X_8X)
	if ((UARTPort == 1) || (UARTPort == 4)) {
		/* Set RS485 control to default state */
		pUART->RS485CTRL = 0;
		/* Set RS485 delay timer to default state */
		pUART->RS485DLY = 0;
		/* Set RS485 addr match to default state */
		pUART->RS485ADRMATCH = 0;
	}
#else
	/* Set RS485 control to default state */
	pUART->RS485CTRL = 0;
	/* Set RS485 delay timer to default state */
	pUART->RS485DLY = 0;
	/* Set RS485 addr match to default state */
	pUART->RS485ADRMATCH = 0;
#endif

#if defined(CHIP_LPC175X_6X) || defined(CHIP_LPC177X_8X) || defined(CHIP_LPC407X_8X) || defined(CHIP_LPC18XX) || \
	defined(CHIP_LPC43XX)
	if (UARTPort == 1) {
#endif
	/* Set Modem Control to default state */
	pUART->MCR = 0;
	/*Dummy Reading to Clear Status */
	tmp = pUART->MSR;
#if defined(CHIP_LPC175X_6X) || defined(CHIP_LPC177X_8X) || defined(CHIP_LPC407X_8X) || defined(CHIP_LPC18XX) || \
	defined(CHIP_LPC43XX)
}

#endif

	/* Dummy reading */
	tmp = pUART->LSR;
	/* Set Line Control register ---------------------------- */
	//	pUART->LCR = UART_DATABIT_8  |                 /* Default: 8N1 */
	//					UART_STOPBIT_1  |
	//					UART_PARITY_NONE;
	pUART->FDR = 0x10;								/* No fractional divider */
}

/* De-initializes the UARTx peripheral registers to their default reset values */
void IP_UART_DeInit(IP_USART_001_T *pUART, IP_UART_ID_T UARTPort)
{
	/* For debug mode */

	IP_UART_TxCmd(pUART, UARTPort, DISABLE);

	switch (UARTPort) {
	default:
		break;
	}
}

// FIXME - this function doesn't work correctly and is too big
/* Determines best dividers to get a target clock rate */
Status IP_UART_SetBaud(IP_USART_001_T *pUART, uint32_t baudrate, uint32_t uClk)
{
	Status errorStatus = ERROR;

	uint32_t d, m, bestd, bestm, tmp;
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

			if (( tmp < 1) || ( tmp > 65536) ) {/* Out of range */
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
		}	/* end of inner for loop */

		if (best_error == 0) {
			break;
		}
	}	/* end of outer for loop  */

	if (best_divisor == 0) {
		return ERROR;					/* can not find best match */

	}
	recalcbaud = (uClk >> 4) * bestm / (best_divisor * (bestm + bestd));

	/* reuse best_error to evaluate baud error */
	if (baudrate > recalcbaud) {
		best_error = baudrate - recalcbaud;
	}
	else {best_error = recalcbaud - baudrate; }

	best_error = best_error * 100 / baudrate;

	if (best_error < UART_ACCEPTED_BAUDRATE_ERROR) {
		pUART->LCR |= UART_LCR_DLAB_EN;
		// FIXME - this is not how DLL and DLM work - they do not add
		pUART->DLM = UART_LOAD_DLM(best_divisor);
		pUART->DLL = UART_LOAD_DLL(best_divisor);
		/* Then reset DLAB bit */
		pUART->LCR &= (~UART_LCR_DLAB_EN) & UART_LCR_BITMASK;
		pUART->FDR = (UART_FDR_MULVAL(bestm)	\
					  | UART_FDR_DIVADDVAL(bestd)) & UART_FDR_BITMASK;
		errorStatus = SUCCESS;
	}

	return errorStatus;
}

/* Configure data width, parity mode and stop bits */
void IP_UART_ConfigData(IP_USART_001_T *pUART,
						IP_UART_DATABIT_T Databits,
						IP_UART_PARITY_T Parity,
						IP_UART_STOPBIT_T Stopbits)
{
	uint32_t tmp = (pUART->LCR & (UART_LCR_DLAB_EN | UART_LCR_BREAK_EN)) & UART_LCR_BITMASK;
	tmp |= (uint32_t) Databits | (uint32_t) Parity | (uint32_t) Stopbits;
	pUART->LCR = (uint8_t) (tmp & UART_LCR_BITMASK);
}

/* UART Send/Recieve functions -------------------------------------------------*/
/* Transmit a single data through UART peripheral */
Status IP_UART_SendByte(IP_USART_001_T *pUART, uint8_t data)
{
	if (!(pUART->LSR & UART_LSR_THRE)) {
		return ERROR;
	}
	pUART->THR = data & UART_THR_MASKBIT;
	return SUCCESS;
}

/* Receive a single data from UART peripheral */
Status IP_UART_ReceiveByte(IP_USART_001_T *pUART, uint8_t *Data)
{
	if (!(pUART->LSR & UART_LSR_RDR)) {
		return ERROR;
	}
	*Data = (uint8_t) (pUART->RBR & UART_RBR_MASKBIT);
	return SUCCESS;
}

/* Send a block of data via UART peripheral */
uint32_t IP_UART_Send(IP_USART_001_T *pUART, uint8_t *txbuf, uint32_t buflen, TRANSFER_BLOCK_T flag)
{
	uint32_t bToSend, bSent, timeOut;	// , fifo_cnt;
	uint8_t *pChar = txbuf;

	bToSend = buflen;

	/* blocking mode */
	if (flag == BLOCKING) {
		bSent = 0;
		while (bToSend) {
			timeOut = UART_BLOCKING_TIMEOUT;
			/* Wait for THR empty with timeout */
			while (!(IP_UART_SendByte(pUART, *pChar))) {
				if (timeOut == 0) {
					break;
				}
				timeOut--;
			}
			/* Time out! */
			if (timeOut == 0) {
				break;
			}
			pChar++;
			bToSend--;
			bSent++;
		}
	}
	/* None blocking mode */
	else {
		bSent = 0;
		while (bToSend) {
			if (!(IP_UART_SendByte(pUART, *pChar))) {
				break;
			}
			pChar++;
			bToSend--;
			bSent++;
		}
	}
	return bSent;
}

/* Receive a block of data via UART peripheral */
uint32_t IP_UART_Receive(IP_USART_001_T *pUART, uint8_t *rxbuf, uint32_t buflen, TRANSFER_BLOCK_T flag)
{
	uint32_t bToRecv, bRecv, timeOut;
	uint8_t *pChar = rxbuf;

	bToRecv = buflen;

	/* Blocking mode */
	if (flag == BLOCKING) {
		bRecv = 0;
		while (bToRecv) {
			timeOut = UART_BLOCKING_TIMEOUT;
			while (!(IP_UART_ReceiveByte(pUART, pChar))) {
				if (timeOut == 0) {
					break;
				}
				timeOut--;
			}
			/* Time out! */
			if (timeOut == 0) {
				break;
			}
			pChar++;
			bToRecv--;
			bRecv++;
		}
	}
	/* None blocking mode */
	else {
		bRecv = 0;
		while (bToRecv) {
			if (!(IP_UART_ReceiveByte(pUART, pChar))) {
				break;
			}
			else {
				pChar++;
				bRecv++;
				bToRecv--;
			}
		}
	}
	return bRecv;
}

/* Enable or disable specified UART interrupt */
void IP_UART_IntConfig(IP_USART_001_T *pUART, IP_UART_INT_T UARTIntCfg, FunctionalState NewState)
{
	uint32_t tmp = 0;

	switch (UARTIntCfg) {
	case UART_INTCFG_RBR:
		tmp = UART_IER_RBRINT_EN;
		break;

	case UART_INTCFG_THRE:
		tmp = UART_IER_THREINT_EN;
		break;

	case UART_INTCFG_RLS:
		tmp = UART_IER_RLSINT_EN;
		break;

	case UART_INTCFG_MS:
		tmp = UART_IER_MSINT_EN;
		break;

	case UART_INTCFG_CTS:
		tmp = UART_IER_CTSINT_EN;
		break;

	case UART_INTCFG_ABEO:
		tmp = UART_IER_ABEOINT_EN;
		break;

	case UART_INTCFG_ABTO:
		tmp = UART_IER_ABTOINT_EN;
		break;
	}

	if (NewState == ENABLE) {
		pUART->IER |= tmp;
	}
	else {

		pUART->IER &= (~tmp) & UART_IER_BITMASK;

	}
}

/* Get Source Interrupt */
uint32_t IP_UART_IntGetStatus(IP_USART_001_T *pUART)
{
	return (pUART->IIR) & UART_IIR_BITMASK;
}

/* Force BREAK character on UART line, output pin UARTx TXD is forced to logic 0 */
void IP_UART_ForceBreak(IP_USART_001_T *pUART)
{
	pUART->LCR |= UART_LCR_BREAK_EN;
}

/* Get current value of Line Status register in UART peripheral */
uint8_t IP_UART_GetLineStatus(IP_USART_001_T *pUART)
{
	return (pUART->LSR) & UART_LSR_BITMASK;
}

/* Check whether if UART is busy or not */
FlagStatus IP_UART_CheckBusy(IP_USART_001_T *pUART)
{
	if (pUART->LSR & UART_LSR_TEMT) {
		return RESET;
	}
	else {
		return SET;
	}
}

/* Enable/Disable transmission on UART TxD pin */
void IP_UART_TxCmd(IP_USART_001_T *pUART, IP_UART_ID_T UARTPort, FunctionalState NewState)
{
	if (NewState == ENABLE) {
#if defined(CHIP_LPC177X_8X) || defined(CHIP_LPC407X_8X) || defined(CHIP_LPC18XX) || defined(CHIP_LPC43XX)
		switch (UARTPort) {
#if defined(CHIP_LPC177X_8X) || defined(CHIP_LPC407X_8X)
		case 4:
#else
		case 0:
		case 2:
		case 3:
#endif
			pUART->TER2 = UART_TER2_TXEN;
			break;

		default:
			break;
		}
#else
		pUART->TER1 = UART_TER1_TXEN;
#endif
	}
	else {
#if defined(CHIP_LPC177X_8X) || defined(CHIP_LPC407X_8X) || defined(CHIP_LPC18XX) || defined(CHIP_LPC43XX)
		switch (UARTPort) {
#if defined(CHIP_LPC177X_8X) || defined(CHIP_LPC407X_8X)
		case 4:
#else
		case 0:
		case 2:
		case 3:
#endif
			pUART->TER2 &= (~UART_TER2_TXEN) & UART_TER2_BITMASK;
			break;

		default:
			break;
		}
#else
		pUART->TER1 &= (~UART_TER1_TXEN) & UART_TER1_BITMASK;
#endif
	}
}

/* UART FIFO functions ----------------------------------------------------------*/
/* Configure FIFO function on selected UART peripheral */
void IP_UART_FIFOConfig(IP_USART_001_T *pUART, UART_FIFO_CFG_T *FIFOCfg)
{
	uint8_t tmp = 0;

	tmp |= UART_FCR_FIFO_EN;
	switch (FIFOCfg->FIFO_Level) {
	case UART_FIFO_TRGLEV0:
		tmp |= UART_FCR_TRG_LEV0;
		break;

	case UART_FIFO_TRGLEV1:
		tmp |= UART_FCR_TRG_LEV1;
		break;

	case UART_FIFO_TRGLEV2:
		tmp |= UART_FCR_TRG_LEV2;
		break;

	case UART_FIFO_TRGLEV3:
	default:
		tmp |= UART_FCR_TRG_LEV3;
		break;
	}

	if (FIFOCfg->FIFO_ResetTxBuf == ENABLE) {
		tmp |= UART_FCR_TX_RS;
	}
	if (FIFOCfg->FIFO_ResetRxBuf == ENABLE) {
		tmp |= UART_FCR_RX_RS;
	}
	if (FIFOCfg->FIFO_DMAMode == ENABLE) {
		tmp |= UART_FCR_DMAMODE_SEL;
	}

	/* write to FIFO control register */
	pUART->FCR = tmp & UART_FCR_BITMASK;
}

/* Fills each UART_FIFOInitStruct member with its default value */
void IP_UART_FIFOConfigStructInit(UART_FIFO_CFG_T *UART_FIFOInitStruct)
{
	UART_FIFOInitStruct->FIFO_DMAMode = DISABLE;
	UART_FIFOInitStruct->FIFO_Level = UART_FIFO_TRGLEV0;
	UART_FIFOInitStruct->FIFO_ResetRxBuf = ENABLE;
	UART_FIFOInitStruct->FIFO_ResetTxBuf = ENABLE;
}

/* Start/Stop Auto Baudrate activity */
void IP_UART_ABCmd(IP_USART_001_T *pUART, UART_AB_CFG_T *ABConfigStruct, FunctionalState NewState)
{
	if (NewState == ENABLE) {
		/* Clear DLL and DLM value */
		pUART->LCR |= UART_LCR_DLAB_EN;
		pUART->DLL = 0;
		pUART->DLM = 0;
		pUART->LCR &= ~UART_LCR_DLAB_EN;

		/* FDR value must be reset to default value */
		pUART->FDR = 0x10;

		if (ABConfigStruct->ABMode == UART_AUTOBAUD_MODE1) {
			pUART->ACR = UART_ACR_START | UART_ACR_MODE;
		}

		if (ABConfigStruct->AutoRestart == ENABLE) {
			pUART->ACR = UART_ACR_START | UART_ACR_AUTO_RESTART;
		}
	}
	else {
		pUART->ACR = 0;
	}
}

/* Clear Autobaud Interrupt Pending */
void IP_UART_ABClearIntPending(IP_USART_001_T *pUART, IP_UART_INT_STATUS_T ABIntType)
{
	pUART->ACR |= ABIntType;
}
