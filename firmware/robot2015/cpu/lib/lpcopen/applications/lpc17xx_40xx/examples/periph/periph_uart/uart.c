/*
 * @brief UART example
 * This example show how to use the UART in 3 modes : Polling, Interrupt and DMA
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

/** @defgroup EXAMPLES_PERIPH_17XX40XX_UART LPC17xx/40xx UART example
 * @ingroup EXAMPLES_PERIPH_17XX40XX
 * <b>Example description</b><br>
 * The UART example shows how to use the UART in polling, interrupt, and
 * DMA modes.<br>
 *
 * To use the example, connect a serial cable to the board's RS232/UART port and
 * start a terminal program to monitor the port.  The terminal program on the host
 * PC should be setup for Baudrate-8-N-1 (Baudrate is optional).
 * After a reset condition, press 'a' or 'A' to start Auto-baudrate mode. Once Auto
 * baud rate mode completed, welcome menu is printed on terminal. Change modes by
 * selecting the option from the terminal.<br>
 *
 * <b>Special connection requirements</b><br>
 * - Embedded Artists' LPC1788 Developer's Kit:<br>
 * - Embedded Artists' LPC4088 Developer's Kit:<br>
 * If using UART2: Connects UART pins to COM Port (J17)<br>
 *  JP6-1-2: OFF:<br>
 *  JP6-3-4: OFF:<br>
 *  JP6-5-6: ON:<br>
 *  JP12: 2-3<br>
 *  JP13: 2-3<br>
 *  Other jumpers: Default<br>
 * - LPCXpresso LPC1769:<br>
 * Need to connect with base board for using RS232/UART port.<br>
 *
 * <b>Build procedures:</b><br>
 * @ref LPCOPEN_17XX40XX_BUILDPROCS_KEIL<br>
 * @ref LPCOPEN_17XX40XX_BUILDPROCS_IAR<br>
 * @ref LPCOPEN_17XX40XX_BUILDPROCS_XPRESSO<br>
 *
 * <b>Supported boards and board setup:</b><br>
 * @ref LPCOPEN_17XX40XX_BOARD_EA1788<br>
 * @ref LPCOPEN_17XX40XX_BOARD_EA4088<br>
 * @ref LPCOPEN_17XX40XX_BOARD_XPRESSO_1769<br>
 *
 * <b>Submitting LPCOpen issues:</b><br>
 * @ref LPCOPEN_COMMUNITY
 * @{
 */

/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/

#if defined(BOARD_EA_DEVKIT_17884088)
#define UARTNum 0
#elif defined(BOARD_NXP_XPRESSO_1769)
#define UARTNum 3
#else
#error No UART selected for undefined board
#endif

#if (UARTNum == 0)
#define LPC_UART LPC_UART0
#define UARTx_IRQn  UART0_IRQn
#define UARTx_IRQHandler UART0_IRQHandler
#define _GPDMA_CONN_UART_Tx GPDMA_CONN_UART0_Tx
#define _GPDMA_CONN_UART_Rx GPDMA_CONN_UART0_Rx
#elif (UARTNum == 1)
#define LPC_UART LPC_UART1
#define UARTx_IRQn  UART1_IRQn
#define UARTx_IRQHandler UART1_IRQHandler
#define _GPDMA_CONN_UART_Tx GPDMA_CONN_UART1_Tx
#define _GPDMA_CONN_UART_Rx GPDMA_CONN_UART1_Rx
#elif (UARTNum == 2)
#define LPC_UART LPC_UART2
#define UARTx_IRQn  UART2_IRQn
#define UARTx_IRQHandler UART2_IRQHandler
#define _GPDMA_CONN_UART_Tx GPDMA_CONN_UART2_Tx
#define _GPDMA_CONN_UART_Rx GPDMA_CONN_UART2_Rx
#elif (UARTNum == 3)
#define LPC_UART LPC_UART3
#define UARTx_IRQn  UART3_IRQn
#define UARTx_IRQHandler UART3_IRQHandler
#define _GPDMA_CONN_UART_Tx GPDMA_CONN_UART3_Tx
#define _GPDMA_CONN_UART_Rx GPDMA_CONN_UART3_Rx
#endif

static uint8_t uartABComplete[] = "UART Auto-Baurate synchronized! \n\r";

/* Uart Polling variables and functions declaration */
static uint8_t uartPolling_menu1[] = "Hello NXP Semiconductors \n\r";
static uint8_t uartPolling_menu2[] = "UART polling mode demo \n\r";
static uint8_t uartPolling_menu3[] = "\n\rUART demo terminated!";
static uint8_t uartPolling_menu4[] = "\n\rPress number 1-3 to choose UART running mode:\n\r"
									 "\t 1: Polling Mode \n\r"
									 "\t 2: Interrupt Mode \n\r"
									 "\t 3: DMA Mode \n\r";
static uint8_t uartPolling_menu5[] = "\n\rPolling mode is running now! Please press \'c\' and choose another mode \n\r";

/* Uart Interrupt variables and functions declaration */
static uint8_t uart_interrupt_menu[] =
	"UART Interrupt mode demo ! \n\rPress '1' to '4' to display 4 menus \n\rPress 'x'to exist uart interrupt mode \n\r";
static uint8_t uart_interrupt_menu1[] = "UART interrupt menu 1 \n\r";
static uint8_t uart_interrupt_menu2[] = "UART interrupt menu 2 \n\r";
static uint8_t uart_interrupt_menu3[] = "UART interrupt menu 3 \n\r";
static uint8_t uart_interrupt_menu4[] = "UART interrupt menu 4 \n\r";
/* static uint8_t rxUartIntBuf[1]; */

#define DMA_TIMEOUT 0xA000000

/* DMA variables and functions declaration */
static uint8_t dmaChannelNumTx, dmaChannelNumRx;
static uint8_t uartDMA_menu[] = "Hello NXP Semiconductors (DMA mode)\n\r"
								"UART DMA mode demo ! Please type 'hello NXP' to return\n\r";

static volatile uint32_t channelTC;	/* Terminal Counter flag for Channel */
static volatile uint32_t channelTCErr;
static FunctionalState  isDMATx = ENABLE;

/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/

/*****************************************************************************
 * Private functions
 ****************************************************************************/

/* Initialize DMA for UART, enable DMA controller and enable DMA interrupt */
static void App_DMA_Init(void)
{
	/* Initialize GPDMA controller */
	Chip_GPDMA_Init(LPC_GPDMA);
	/* Setting GPDMA interrupt */
	NVIC_DisableIRQ(DMA_IRQn);
	NVIC_SetPriority(DMA_IRQn, ((0x01 << 3) | 0x01));
	NVIC_EnableIRQ(DMA_IRQn);
}

/* DeInitialize DMA for UART, free transfer channels and disable DMA interrupt */
static void App_DMA_DeInit(void)
{
	Chip_DMA_Stop(LPC_GPDMA, dmaChannelNumTx);
	Chip_DMA_Stop(LPC_GPDMA, dmaChannelNumRx);
	NVIC_DisableIRQ(DMA_IRQn);
}

/* DMA routine for example_uart */
static void App_DMA_Test(void)
{
	uint8_t receiveBuffer[16];

	App_DMA_Init();
	dmaChannelNumTx = Chip_DMA_GetFreeChannel(LPC_GPDMA, _GPDMA_CONN_UART_Tx);

	isDMATx = ENABLE;
	channelTC = channelTCErr = 0;
	Chip_DMA_Transfer(LPC_GPDMA, dmaChannelNumTx,
					  (uint32_t) &uartDMA_menu[0],
					  _GPDMA_CONN_UART_Tx,
					  GPDMA_TRANSFERTYPE_M2P_CONTROLLER_DMA,
					  sizeof(uartDMA_menu));
	while (!channelTC) {}

	dmaChannelNumRx = Chip_DMA_GetFreeChannel(LPC_GPDMA, _GPDMA_CONN_UART_Rx);
	isDMATx = DISABLE;
	channelTC = channelTCErr = 0;
	Chip_DMA_Transfer(LPC_GPDMA, dmaChannelNumRx,
					  _GPDMA_CONN_UART_Rx,
					  (uint32_t) &receiveBuffer[0],
					  GPDMA_TRANSFERTYPE_P2M_CONTROLLER_DMA,
					  10);
	while (!channelTC) {}

	isDMATx = ENABLE;
	channelTC = channelTCErr = 0;
	Chip_DMA_Transfer(LPC_GPDMA, dmaChannelNumTx,
					  (uint32_t) &receiveBuffer[0],
					  _GPDMA_CONN_UART_Tx,
					  GPDMA_TRANSFERTYPE_M2P_CONTROLLER_DMA,
					  10);
	while (!channelTC) {}

	App_DMA_DeInit();
}

/* Print Welcome Screen Menu subroutine by Interrupt mode */
static void Print_Menu_Interrupt(LPC_USART_T *UARTx)
{
	uint32_t tmp, tmp2;
	uint8_t *pDat;

	tmp = sizeof(uart_interrupt_menu);
	tmp2 = 0;
	pDat = (uint8_t *) &uart_interrupt_menu[0];
	while (tmp) {
		tmp2 = Chip_UART_Interrupt_Transmit(UARTx, pDat, tmp);
		pDat += tmp2;
		tmp -= tmp2;
	}
}

/* Initialize Interrupt for UART */
static void App_Interrupt_Init(void)
{
	/* Enable UART Rx interrupt */
	Chip_UART_IntConfig(LPC_UART, UART_INTCFG_RBR, ENABLE);
	/* Enable UART line status interrupt */
	Chip_UART_IntConfig(LPC_UART, UART_INTCFG_RLS, ENABLE);
	/*
	 * Do not enable transmit interrupt here, since it is handled by
	 * UART_Send() function, just to reset Tx Interrupt state for the
	 * first time
	 */
	Chip_UART_InitRingBuffer(LPC_UART);
	/* Enable Interrupt for UART channel */
	/* Priority = 1 */
	NVIC_SetPriority(UARTx_IRQn, 1);
	/* Enable Interrupt for UART channel */
	NVIC_EnableIRQ(UARTx_IRQn);
}

/* DeInitialize Interrupt for UART */
static void App_Interrupt_DeInit(void)
{
	/* Disable UART Rx interrupt */
	Chip_UART_IntConfig(LPC_UART, UART_INTCFG_RBR, DISABLE);
	/* Disable UART line status interrupt */
	Chip_UART_IntConfig(LPC_UART, UART_INTCFG_RLS, DISABLE);
	/* Disable Interrupt for UART channel */
	NVIC_DisableIRQ(UARTx_IRQn);
}

/* Interrupt routine for example_uart */
static void App_Interrupt_Test(void)
{
	uint8_t isExit = 0, userInput;
	uint32_t len;
	App_Interrupt_Init();

	/* Print out uart interrupt menu */
	Print_Menu_Interrupt(LPC_UART);

	while (!isExit) {
		len = 0;
		while (len == 0) {
			len = Chip_UART_Interrupt_Receive(LPC_UART, &userInput, 1);
		}
		if (userInput == '1') {
			Chip_UART_Interrupt_Transmit(LPC_UART, (uint8_t *) &uart_interrupt_menu1[0], sizeof(uart_interrupt_menu1));
		}
		else if (userInput == '2') {
			Chip_UART_Interrupt_Transmit(LPC_UART, (uint8_t *) &uart_interrupt_menu2[0], sizeof(uart_interrupt_menu2));
		}
		else if (userInput == '3') {
			Chip_UART_Interrupt_Transmit(LPC_UART, (uint8_t *) &uart_interrupt_menu3[0], sizeof(uart_interrupt_menu3));
		}
		else if (userInput == '4') {
			Chip_UART_Interrupt_Transmit(LPC_UART, (uint8_t *) &uart_interrupt_menu4[0], sizeof(uart_interrupt_menu4));
		}
		else if (( userInput == 'x') || ( userInput == 'X') ) {
			isExit = 1;
		}
	}
	App_Interrupt_DeInit();
}

/* Print Welcome menu by Polling mode */
static void Print_Menu_Polling(void)
{
	Chip_UART_Send(LPC_UART, uartPolling_menu1, sizeof(uartPolling_menu1), BLOCKING);
	Chip_UART_Send(LPC_UART, uartPolling_menu2, sizeof(uartPolling_menu2), BLOCKING);
	Chip_UART_Send(LPC_UART, uartPolling_menu5, sizeof(uartPolling_menu5), BLOCKING);
}

/*****************************************************************************
 * Public functions
 ****************************************************************************/

/**
 * @brief	GPDMA interrupt handler sub-routine
 * @return	Nothing
 */
void DMA_IRQHandler(void)
{
	uint8_t dmaChannelNum;
	if (isDMATx) {
		dmaChannelNum = dmaChannelNumTx;
	}
	else {
		dmaChannelNum = dmaChannelNumRx;
	}
	if (Chip_DMA_Interrupt(LPC_GPDMA, dmaChannelNum) == SUCCESS) {
		channelTC++;
	}
	else {
		channelTCErr++;
	}
}

/**
 * @brief	UART interrupt handler sub-routine
 * @return	Nothing
 */
void UARTx_IRQHandler(void)
{
	Chip_UART_Interrupt_Handler(LPC_UART);
}

/**
 * @brief	Main UART program body
 * @return	Always returns -1
 */
int main(void)
{
	FlagStatus exitflag;
	uint8_t buffer[10];
	int ret = 0;
	uint32_t len;

	/* UART FIFO configuration Struct variable */
	UART_FIFO_CFG_T UARTFIFOConfigStruct;
	/* Auto baudrate configuration structure */
	UART_AB_CFG_T ABConfig;

	Board_Init();
	Board_UART_Init(LPC_UART);

#if (UARTNum != 0)
	Chip_UART_Init(LPC_UART);
	Chip_UART_SetBaud(LPC_UART, 115200);
	Chip_UART_ConfigData(LPC_UART, UART_DATABIT_8, UART_PARITY_NONE, UART_STOPBIT_1);	/* Default 8-N-1 */

	/* Enable UART Transmit */
	Chip_UART_TxCmd(LPC_UART, ENABLE);
#endif

	Chip_UART_FIFOConfigStructInit(LPC_UART, &UARTFIFOConfigStruct);

	/* Enable DMA mode in UART */
	UARTFIFOConfigStruct.FIFO_DMAMode = ENABLE;
	/* Initialize FIFO for UART0 peripheral */
	Chip_UART_FIFOConfig(LPC_UART, &UARTFIFOConfigStruct);

	/* Enable UART End of Auto baudrate interrupt */
	Chip_UART_IntConfig(LPC_UART, UART_INTCFG_ABEO, ENABLE);
	/* Enable UART Auto baudrate timeout interrupt */
	Chip_UART_IntConfig(LPC_UART, UART_INTCFG_ABTO, ENABLE);

	/* preemption = 1, sub-priority = 1 */
	NVIC_SetPriority(UARTx_IRQn, 1);
	/* Enable Interrupt for UART0 channel */
	NVIC_EnableIRQ(UARTx_IRQn);

	/* ---------------------- Auto baud rate section ----------------------- */
	/* Configure Auto baud rate mode */
	ABConfig.ABMode = UART_AUTOBAUD_MODE0;
	ABConfig.AutoRestart = ENABLE;

	/* Start auto baudrate mode */
	Chip_UART_ABCmd(LPC_UART, &ABConfig, ENABLE);

	/* Loop until auto baudrate mode complete */
	while (Chip_UART_GetABEOStatus(LPC_UART) == RESET) {}

	Chip_UART_Send(LPC_UART, uartABComplete, sizeof(uartABComplete), BLOCKING);

	/* Disable UART Interrupt */
	NVIC_DisableIRQ(UARTx_IRQn);

	/* Print welcome screen */
	Print_Menu_Polling();

	exitflag = RESET;
	/* Read some data from the buffer */
	while (exitflag == RESET) {
		len = 0;
		while (len == 0) {
			len = Chip_UART_Receive(LPC_UART, buffer, 1, NONE_BLOCKING);
		}
		if (buffer[0] == 27) {
			/* ESC key, set exit flag */
			Chip_UART_Send(LPC_UART, uartPolling_menu3, sizeof(uartPolling_menu3), BLOCKING);
			ret = -1;
			exitflag = SET;
		}
		else if (buffer[0] == 'c') {
			Chip_UART_Send(LPC_UART, uartPolling_menu4, sizeof(uartPolling_menu4), BLOCKING);
			len = 0;
			while (len == 0) {
				len = Chip_UART_Receive(LPC_UART, buffer, sizeof(buffer), NONE_BLOCKING);
				if ((buffer[0] != '1') && (buffer[0] != '2') && (buffer[0] != '3')) {
					len = 0;
				}
			}
			switch (buffer[0]) {
			case '1':		/* Polling Mode */
				Chip_UART_Send(LPC_UART, uartPolling_menu5, sizeof(uartPolling_menu5), BLOCKING);
				break;

			case '2':		/* Interrupt Mode */
				ret = 2;
				/* Exitflag = SET; */
				App_Interrupt_Test();
				Print_Menu_Polling();
				break;

			case '3':		/* DMA mode */
				ret = 3;
				App_DMA_Test();
				Print_Menu_Polling();
				break;
			}
		}
	}

	/* Wait for current transmission complete - THR must be empty */
	while (Chip_UART_CheckBusy(LPC_UART) == SET) {}

	/* DeInitialize UART0 peripheral */
	Chip_UART_DeInit(LPC_UART);

	return ret;
}

/**
 * @}
 */
