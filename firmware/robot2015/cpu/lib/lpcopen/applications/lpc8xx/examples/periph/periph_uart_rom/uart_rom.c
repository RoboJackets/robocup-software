/*
 * @brief Universal Asynchronous Receiver/Transmitter API in ROM (USART API ROM) example
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

/** @defgroup EXAMPLES_PERIPH_8XX_UART_API_ROM LPC8xx USART example using UART ROM API functions
 * @ingroup EXAMPLES_PERIPH_8XX
 * <b>Example description</b><br>
 * The UART_ROM example demonstrates using the USART API ROM functions.<br>
 *
 * <b>Special connection requirements</b><br>
 * There are no special connection requirements for this example.<br>
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

#define APP_UART_RECEIVE_BUGGER_SIZE 9

static UART_HANDLE_T *uart;

static uint32_t umem[UART_ROM_MEM_SIZE];
static const char welcome[] = "LPC812 XPresso Board USART API ROM Example.\0";
static const char Polling_info[] = "UART Polling mode demo! Please type 'hello NXP' to continue.\0";
static const char Interrupt_info[] =
	"\r\nUART Interrupt mode demo! Type a character to be echoed back or \'x\' to exit.\0";
static const char terminate[] = "\r\nDemo is terminated. Thanks for using.\0";

static char recv_buf[APP_UART_RECEIVE_BUGGER_SIZE];
static volatile uint8_t byte, test_stop = 0;
static volatile uint32_t tx_Done = 0;

/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/

/*****************************************************************************
 * Private functions
 ****************************************************************************/

static void rx_callback(uint32_t err_code, uint32_t n);

static void tx_callback(uint32_t err_code, uint32_t n);

static void App_error_routine()
{
	Board_LED_Set(0, true);	/* Test fail */
	__WFE();
}

static void App_Polling_Send(const char *send_data, uint32_t lenght)
{
	UART_PARAM_T param;
	param.buffer = (uint8_t *) send_data;			/* the data to send */
	param.size = lenght;							/* size of the data */
	param.transfer_mode = TX_MODE_SZERO_SEND_CRLF;	/* send until zero-terminator of string + append CRLF */
	param.driver_mode = DRIVER_MODE_POLLING;		/* simple "polled" (blocking) transfer */
	/* transmit the message */
	if (LPC_UART_API->uart_put_line(uart, &param)) {
		App_error_routine();
	}
}

static void App_Polling_Receive(char *receive_buffer, uint32_t lenght)
{
	UART_PARAM_T param;
	param.buffer = (uint8_t *) receive_buffer;		/* the buffer for receiving */
	param.size = lenght;							/* size of the buffer */
	param.transfer_mode = RX_MODE_LF_RECVD;			/* receive until get LF character of buffer is full */
	param.driver_mode = DRIVER_MODE_POLLING;		/* simple "polled" (blocking) transfer */
	/* receive the message */
	if (LPC_UART_API->uart_get_line(uart, &param)) {
		App_error_routine();
	}
}

static void App_Interrupt_Send(const char *send_data, uint32_t lenght)
{
	UART_PARAM_T param;
	param.buffer = (uint8_t *) send_data;					/* the data to send */
	param.size = lenght;									/* size of the data */
	param.transfer_mode = TX_MODE_SZERO_SEND_CRLF;			/* send until zero-terminator of string + append CRLF */
	param.driver_mode = DRIVER_MODE_INTERRUPT;				/* Interrupt (non-blocking) transfer */
	param.callback_func_pt = (UART_CALLBK_T) tx_callback;	/* call back function when transmit is completed */
	/* transmit the message */
	if (LPC_UART_API->uart_put_line(uart, &param)) {
		App_error_routine();
	}
}

static void App_UART_API_Init()
{
	uint32_t frg_mult;
	/* Initialize the UARTs, Get the UART memory size needed */
	volatile uint32_t mem = LPC_UART_API->uart_get_mem_size();
	/* Configure the UART settings */
	UART_CONFIG_T cfg = {0,	/* U_PCLK frequency in Hz */
						 115200,						/* Baud Rate in Hz */
						 1,								/* 8N1 */
						 0,								/* Asynchronous Mode */
						 NO_ERR_EN						/* Enable No Errors */
	};
	cfg.sys_clk_in_hz = Chip_Clock_GetMainClockRate();
	/* Perform a sanity check on the storage allocation */
	if (UART_ROM_MEM_SIZE < (mem / sizeof(uint32_t))) {
		while (1) {}
	}


	
	/* Setup the UART */
	uart = LPC_UART_API->uart_setup((uint32_t) LPC_USART0, (uint8_t *) umem);

	/* Check the API return value for a valid handle */
	if (uart != NULL) {
		/* initialize the UART with the configuration parameters */
		frg_mult = LPC_UART_API->uart_init(uart, &cfg);	/* API returns FRG MULT for cfg'd baud rate */
		if (frg_mult) {
			Chip_SYSCTL_SetUSARTFRGDivider(0xFF);	/* value 0xFF should be always used */
			Chip_SYSCTL_SetUSARTFRGMultiplier(frg_mult);
		}
	}
}

static void rx_callback(uint32_t err_code, uint32_t n)
{
	App_Interrupt_Send( (char *) &byte, 1);	/* echo the character back */
	if (byte == 'x') {
		test_stop = 1;
	}
}

static void tx_callback(uint32_t err_code, uint32_t n)
{
	tx_Done = 1;
}

/*****************************************************************************
 * Public functions
 ****************************************************************************/

/**
 * @brief	UART0 ISR handler
 * @return	Nothing
 */
void UART0_IRQHandler(void)
{
	LPC_UART_API->uart_isr(uart);
}

/**
 * @brief	Main Function
 * @return	Nothing
 */

int main(void)
{
    volatile int k = 1;
    
	UART_PARAM_T param;

	Board_Init();

	Board_UART_Init(LPC_USART0);

	App_UART_API_Init();/* Initialize the UARTs 0 */

	/* transmit the "welcome" message */
	App_Polling_Send(welcome, sizeof(welcome));
	App_Polling_Send(Polling_info, sizeof(Polling_info));
	App_Polling_Receive(recv_buf, sizeof(recv_buf));
	App_Polling_Send(recv_buf, sizeof(recv_buf));

	/* Enable the IRQ for the UART */
	NVIC_DisableIRQ(UART0_IRQn);
	NVIC_ClearPendingIRQ(UART0_IRQn);
	NVIC_EnableIRQ(UART0_IRQn);

	App_Interrupt_Send(Interrupt_info, sizeof(Interrupt_info));
	/* waiting for transmit is completed */
	while (!tx_Done) {}

	param.buffer = (uint8_t *) &byte;
	param.size = sizeof(byte);
	param.transfer_mode = RX_MODE_BUF_FULL;
	param.driver_mode = DRIVER_MODE_INTERRUPT;
	param.callback_func_pt = (UART_CALLBK_T) rx_callback;
	while (!test_stop) {
		LPC_UART_API->uart_get_line(uart, &param);
		__WFE();
	}

	NVIC_DisableIRQ(UART0_IRQn);
	App_Polling_Send(terminate, sizeof(terminate));

    while(k);
    
	return 0;
}

/**
 * @}
 */
