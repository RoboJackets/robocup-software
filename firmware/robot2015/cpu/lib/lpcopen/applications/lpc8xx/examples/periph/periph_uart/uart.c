/*
 * @brief Universal Asynchronous Receiver/Transmitter (UART) example
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

#include "board.h"

/** @defgroup EXAMPLES_PERIPH_8XX_UART LPC8xx UART example
 * @ingroup EXAMPLES_PERIPH_8XX
 * <b>Example description</b><br>
 * The UART example demonstrates the transfer of data using the UART module.
 * The data transfer will happen in a loop from UART0 to UART1 to
 * UART0. The UART0 will operate in polling mode, while UART1 will
 * work in interrupt mode. The data transmitted will be checked with data
 * recived to verify the data transfer operation. If the data transfer is
 * succesful, the RED LED on the board�will be turned on<br>
 *
 * <b>Special connection requirements</b><br>
 * Connect pins on the LPC812 XPresso board :<br>
 *	TX0	(P_0.4)	(pin 5 on J7)	->	RX1	(P_0.14) (pin 5 on J6)<br>
 *	TX1	(P_0.13) (pin 8 on J6)	->	RX0	(P_0.0) (pin 4 on J7)<br>
 *
 * <b>Build procedures:</b><br>
 * @ref LPCOPEN_8XX_BUILDPROCS_IAR<br>
 * @ref LPCOPEN_8XX_BUILDPROCS_KEIL<br>
 * @ref LPCOPEN_8XX_BUILDPROCS_XPRESSO<br>
 *
 * <b>Supported boards and board setup:</b><br>
 * @ref LPCOPEN_8XX_BOARD_XPRESSO_812<br>
 *
 * <b>Submitting LPCOpen issues:</b><br>
 * @ref LPCOPEN_COMMUNITY
 * @{
 */

/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/

#define BUFFER_SIZE 0xF
#define UART_TEST_DEFAULT_BAUDRATE 115200
#define UART_TEST_REPEAT_NUMBER 0xF

static uint8_t TxBuf0[BUFFER_SIZE];
static uint8_t RxBuf1[BUFFER_SIZE];
static uint8_t RxBuf0[BUFFER_SIZE];
static uint32_t RxBufCnt1 = 0;
static uint32_t TxBufCnt1 = 0;
static volatile bool receiveCompleted = true;
static volatile bool sendCompleted = true;

/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/

/*****************************************************************************
 * Private functions
 ****************************************************************************/

/* Buffer initialisation function */
static void bufferInit(uint8_t seed)
{
	uint32_t i;
	for (i = 0; i < BUFFER_SIZE; i++) {
		TxBuf0[i] = i + seed;
		RxBuf1[i] = 0;
		RxBuf0[i] = 0;
	}
}

/* Buffer check function */
static void bufferCheck()
{
	uint32_t i;
	for (i = 0; i < BUFFER_SIZE; i++) {
		if (TxBuf0[i] != RxBuf0[i]) {
			while (1) {
				/* Data mismatch */
			}
		}
	}
}

/* UART initialistaion function */
static void App_UART_Init(LPC_USART_T *pUART)
{
	Board_UART_Init(pUART);
	Chip_UART_ConfigData(pUART, UART_DATALEN_8, UART_PARITY_NONE, UART_STOPLEN_1);
	Chip_UART_SetBaudRate(pUART, UART_TEST_DEFAULT_BAUDRATE);
	Chip_UART_Init(pUART);
}

/*****************************************************************************
 * Public functions
 ****************************************************************************/

/**
 * @brief	Handle interrupt from UART
 * @return	Nothing
 */
void UART1_IRQHandler(void)
{
	uint8_t ch;
	uint32_t IntStatus = Chip_UART_GetIntStatus(LPC_USART1);

	if (IntStatus & RXRDY_INT) {
		if(receiveCompleted == false) {
			Chip_UART_ReceiveByte(LPC_USART1, &RxBuf1[RxBufCnt1++]);
			if (RxBufCnt1 == BUFFER_SIZE) {
				Chip_UART_IntEnable(LPC_USART1, RXRDY_INT, DISABLE);
				receiveCompleted = true;
				RxBufCnt1 = 0;
			}
		}
		else {
			Chip_UART_ReceiveByte(LPC_USART1, &ch);
		}
	}

	if (IntStatus & TXRDY_INT) {
		if (sendCompleted == false) {
			Chip_UART_SendByte(LPC_USART1, RxBuf1[TxBufCnt1++]);
			if (TxBufCnt1 == BUFFER_SIZE) {
				Chip_UART_IntEnable(LPC_USART1, TXRDY_INT, DISABLE);
				sendCompleted = true;
				TxBufCnt1 = 0;
			}
		}
		else {
			return;
		}

	}
}

/**
 * @brief	Application main function
 * @return	Does not return
 * @note	This function will not return
 */
int main(void)
{
	uint32_t i;
	uint8_t repeat_num = UART_TEST_REPEAT_NUMBER;
	volatile int j = 1;
	uint8_t ch = 0;
	uint32_t IntStatus;

	/* Generic Initialization */
	Board_Init();

	Board_LED_Set(0, false);

	/* Disable UART1 IRQ */
	NVIC_DisableIRQ(UART1_IRQn);

	/* Initialize the UARTs */
	App_UART_Init(LPC_USART0);
	App_UART_Init(LPC_USART1);

	/* Custom Initialization */
	Chip_UART_IntEnable(LPC_USART1, RXRDY_INT, DISABLE);
	Chip_UART_IntEnable(LPC_USART1, TXRDY_INT, DISABLE);
	NVIC_EnableIRQ(UART1_IRQn);

	/* Data transfer loop */
	while (repeat_num--) {

		bufferInit(repeat_num);

		/* Sending from UART0 (polling mode) to UART1 (interrupt mode) */
		receiveCompleted = false;
		Chip_UART_IntEnable(LPC_USART1, RXRDY_INT, ENABLE);
		for (i = 0; i < BUFFER_SIZE; i++) {
			while (Chip_UART_SendByte(LPC_USART0, TxBuf0[i]) != SUCCESS) {}
		}
		while (!receiveCompleted) {}

        /* Clear Rx FIFO */
		Chip_UART_ReceiveByte(LPC_USART0, &ch);

		/* Sending from UART1 (interrupt mode) to UART0 (polling mode) */
		sendCompleted = false;
		Chip_UART_IntEnable(LPC_USART1, TXRDY_INT, ENABLE);
		for (i = 0; i < BUFFER_SIZE; i++) {
			while (Chip_UART_ReceiveByte(LPC_USART0, &RxBuf0[i]) != SUCCESS) {}
		}
		while (!sendCompleted) {}

		bufferCheck();
	}

	NVIC_DisableIRQ(UART1_IRQn);

	/* Test OK - Turn on Red LED */
	Board_LED_Set(0, true);

	/* Should not return */
	while (j) {}

	return 0;
}

/**
 * @}
 */
