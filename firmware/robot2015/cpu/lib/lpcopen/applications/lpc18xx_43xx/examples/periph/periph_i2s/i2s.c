/*
 * @brief I2S example
 * This example show how to use the I2S in 3 modes : Polling, Interrupt and DMA
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

/** @defgroup EXAMPLES_PERIPH_18XX43XX_I2S LPC18xx/43xx I2S example
 * @ingroup EXAMPLES_PERIPH_18XX43XX
 * <b>Example description</b><br>
 * The I2S example shows how to configure I2S and UDA1380 to receive audio signal and
 * play back with three modes: polling, interrupt, and DMA.<br>
 *
 * To use the example, connect a serial cable to the board's RS232/UART port and
 * start a terminal program to monitor the port.  The terminal program on the host
 * PC should be setup for 115K8N1.<br>
 * For boards that has no default UART (ex: NGX Xplorer), by default example will
 * start in DMA mode hence no need to slect anything from terminal.<br>
 * Connect the computer line out to board line-in (using 3.5mm male-to-male cable),
 * plug the headphone/speaker into board line-out. Change modes by selecting the
 * option from the terminal. Play audio on computer and listen from target board!<br>
 *
 * <b>Special connection requirements</b><br>
 * There are no special connection requirements for this example.<br>
 *
 * <b>Build procedures:</b><br>
 * @ref LPCOPEN_18XX43XX_BUILDPROCS_KEIL<br>
 * @ref LPCOPEN_18XX43XX_BUILDPROCS_IAR<br>
 * @ref LPCOPEN_18XX43XX_BUILDPROCS_XPRESSO<br>
 *
 * <b>Supported boards and board setup:</b><br>
 * @ref LPCOPEN_18XX_BOARD_HITEX1850<br>
 * @ref LPCOPEN_43XX_BOARD_HITEX4350<br>
 * @ref LPCOPEN_18XX_BOARD_KEIL1857<br>
 * @ref LPCOPEN_43XX_BOARD_KEIL4357<br>
 * @ref LPCOPEN_18XX_BOARD_NGX1830<br>
 * @ref LPCOPEN_43XX_BOARD_NGX4330<br>
 *
 * <b>Submitting LPCOpen issues:</b><br>
 * @ref LPCOPEN_COMMUNITY
 * @{
 */

/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/

#define BUFFER_FULL 0
#define BUFFER_EMPTY 1
#define BUFFER_AVAILABLE 2

typedef struct ring_buff {
	uint32_t buffer[256];
	uint8_t read_index;
	uint8_t write_index;
} Ring_Buffer_t;

static char WelcomeMenu[] = "\r\nHello NXP Semiconductors \r\n"
							"I2S DEMO : Connect audio headphone out from computer to line-in on tested board to get audio signal\r\n"
							"Please press \'1\' to test Polling mode\r\n"
							"Please press \'2\' to test Interrupt mode\r\n"
							"Please press \'3\' to test DMA mode\r\n"
							"Please press \'x\' to exit test mode\r\n"
							"Please press \'m\' to DISABLE/ENABLE mute\r\n";

static Ring_Buffer_t ring_buffer;

static uint8_t send_flag;
static uint8_t channelTC;
static uint8_t dmaChannelNum_I2S_Tx, dmaChannelNum_I2S_Rx;
static uint8_t dma_send_receive;

/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/

/*****************************************************************************
 * Private functions
 ****************************************************************************/
uint8_t mute_status = 0;

/* Get input from console */
int Con_GetInput(void)
{
#ifdef DEBUG_ENABLE
	return DEBUGIN();
#else
	return '3';
#endif
}

static void mute_toggle()
{
	mute_status = !mute_status;
	if (mute_status) {
		Chip_I2S_EnableMute(LPC_I2S0);
		DEBUGOUT("MUTE ENABLE\n\r");
	}
	else {
		Chip_I2S_DisableMute(LPC_I2S0);
		DEBUGOUT("MUTE DISABLE\n\r"); 
	}
}

/* Get status of the ring buffer */
static uint8_t ring_buff_get_status(Ring_Buffer_t *ring_buff)
{
	if (ring_buff->read_index == ring_buff->write_index) {
		return BUFFER_EMPTY;
	}
	else if (ring_buff->read_index == (ring_buff->write_index) + 1) {
		return BUFFER_FULL;
	}
	else {return BUFFER_AVAILABLE; }
}

/* Interrupt routine for I2S example */
static void App_Interrupt_Test(void)
{
	uint8_t bufferUART, continue_Flag = 1;
	DEBUGOUT("I2S Interrupt mode\r\n");
	Chip_I2S_Int_Cmd(LPC_I2S0, I2S_RX_MODE,    ENABLE, 4);
	Chip_I2S_Int_Cmd(LPC_I2S0, I2S_TX_MODE,    ENABLE, 4);
	NVIC_EnableIRQ(I2S0_IRQn);
	while (continue_Flag) {
		bufferUART = 0xFF;
		bufferUART = Con_GetInput();
		switch (bufferUART) {
		case 'x':
			continue_Flag = 0;
			Chip_I2S_Int_Cmd(LPC_I2S0, I2S_RX_MODE,    DISABLE, 4);
			NVIC_DisableIRQ(I2S0_IRQn);
			DEBUGOUT(WelcomeMenu);
			break;

		case 'm':
			mute_toggle();
			break;

		default:
			break;
		}
	}
}

/* Polling routine for I2S example */
static void App_Polling_Test(void)
{
	uint32_t polling_data = 0;
	uint8_t bufferUART, continue_Flag = 1;
	DEBUGOUT("I2S Polling mode\r\n");
	while (continue_Flag) {
		bufferUART = 0xFF;
		bufferUART = Con_GetInput();
		switch (bufferUART) {
		case 'x':
			continue_Flag = 0;
			DEBUGOUT(WelcomeMenu);
			break;

		case 'm':
			mute_toggle();
			break;

		default:
			break;
		}

		if (Chip_I2S_GetLevel(LPC_I2S0, I2S_RX_MODE) > 0) {
			polling_data = Chip_I2S_Receive(LPC_I2S0);
			send_flag = 1;
		}
		if ((Chip_I2S_GetLevel(LPC_I2S0, I2S_TX_MODE) < 4) && (send_flag == 1)) {
			Chip_I2S_Send(LPC_I2S0, polling_data);
			send_flag = 0;
		}
	}
}

/* DMA routine for I2S example */
static void App_DMA_Test(void)
{
	uint8_t continue_Flag = 1, bufferUART = 0xFF;
	Chip_I2S_DMA_Cmd(LPC_I2S0, I2S_TX_MODE, I2S_DMA_REQUEST_NUMBER_1, ENABLE, 4);
	Chip_I2S_DMA_Cmd(LPC_I2S0, I2S_RX_MODE, I2S_DMA_REQUEST_NUMBER_2, ENABLE, 4);
	/* Initialize GPDMA controller */
	Chip_GPDMA_Init(LPC_GPDMA);
	/* Setting GPDMA interrupt */
	NVIC_DisableIRQ(DMA_IRQn);
	NVIC_SetPriority(DMA_IRQn, ((0x01 << 3) | 0x01));
	NVIC_EnableIRQ(DMA_IRQn);

	dmaChannelNum_I2S_Rx = Chip_DMA_GetFreeChannel(LPC_GPDMA, GPDMA_CONN_I2S_Rx_Channel_1);

	Chip_DMA_Transfer(LPC_GPDMA, dmaChannelNum_I2S_Rx,
					  GPDMA_CONN_I2S_Rx_Channel_1,
					  GPDMA_CONN_I2S_Tx_Channel_0,
					  GPDMA_TRANSFERTYPE_P2P_CONTROLLER_SrcPERIPHERAL,
					  1);

	DEBUGOUT("I2S DMA mode\r\n");
	while (continue_Flag) {
		bufferUART = 0xFF;
		bufferUART = Con_GetInput();
		switch (bufferUART) {
		case 'x':
			continue_Flag = 0;
			Chip_I2S_DMA_Cmd(LPC_I2S0, I2S_RX_MODE, 2, DISABLE, 1);
			Chip_I2S_DMA_Cmd(LPC_I2S0, I2S_TX_MODE, 1, DISABLE, 1);

			Chip_DMA_Stop(LPC_GPDMA, dmaChannelNum_I2S_Rx);
			NVIC_DisableIRQ(DMA_IRQn);
			DEBUGOUT(WelcomeMenu);
			break;

		case 'm':
			mute_toggle();
			break;

		default:
			break;
		}
	}
}

/*****************************************************************************
 * Public functions
 ****************************************************************************/

/**
 * @brief	DMA interrupt handler sub-routine
 * @return	Nothing
 */
void DMA_IRQHandler(void)
{
	if (dma_send_receive == 1) {
		if (Chip_DMA_Interrupt(LPC_GPDMA, dmaChannelNum_I2S_Rx) == SUCCESS) {
			channelTC++;
		}
		else {
			/* Process error here */
		}
	}
	else {
		if (Chip_DMA_Interrupt(LPC_GPDMA, dmaChannelNum_I2S_Tx) == SUCCESS) {
			channelTC++;
		}
		else {
			/* Process error here */
		}
	}
}

/**
 * @brief	I2S0 interrupt handler sub-routine
 * @return	Nothing
 */
void I2S0_IRQHandler(void)
{
	while ((ring_buff_get_status(&ring_buffer) != BUFFER_FULL) && (Chip_I2S_GetLevel(LPC_I2S0, I2S_RX_MODE) > 0)) {
		ring_buffer.buffer[ring_buffer.write_index++] = Chip_I2S_Receive(LPC_I2S0);
	}
	while ((ring_buff_get_status(&ring_buffer) != BUFFER_EMPTY) && (Chip_I2S_GetLevel(LPC_I2S0, I2S_TX_MODE) < 8)) {
		Chip_I2S_Send(LPC_I2S0, ring_buffer.buffer[ring_buffer.read_index++]);
	}
}

/**
 * @brief  Main routine for I2S example
 * @return Nothing
 */
int main(void)
{

	Chip_I2S_Audio_Format_T audio_Confg;
	uint8_t bufferUART, continue_Flag = 1;
	audio_Confg.SampleRate = 48000;
	/* Select audio data is 2 channels (1 is mono, 2 is stereo) */
	audio_Confg.ChannelNumber = 2;
	/* Select audio data is 16 bits */
	audio_Confg.WordWidth = 16;

	Board_Init();
#if defined( __GNUC__ )
	__sys_write(0, "", 0);
#endif

	DEBUGOUT(WelcomeMenu);
	Board_Audio_Init(LPC_I2S0, UDA1380_LINE_IN);

	Chip_I2S_Init(LPC_I2S0);
	Chip_I2S_Config(LPC_I2S0, I2S_RX_MODE, &audio_Confg);
	Chip_I2S_Config(LPC_I2S0, I2S_TX_MODE, &audio_Confg);

	Chip_I2S_Stop(LPC_I2S0, I2S_TX_MODE);
	Chip_I2S_DisableMute(LPC_I2S0);
	Chip_I2S_Start(LPC_I2S0, I2S_TX_MODE);
	send_flag = 0;
	while (continue_Flag) {
		bufferUART = 0xFF;
		bufferUART = Con_GetInput();
		switch (bufferUART) {
		case '1':
			App_Polling_Test();
			break;

		case '2':
			App_Interrupt_Test();
			break;

		case '3':
			App_DMA_Test();
			break;

		case 'x':
			continue_Flag = 0;
			DEBUGOUT("Thanks for using\r\n");
			break;

		default:
			break;
		}
	}
	return 0;
}

/**
 * @}
 */
