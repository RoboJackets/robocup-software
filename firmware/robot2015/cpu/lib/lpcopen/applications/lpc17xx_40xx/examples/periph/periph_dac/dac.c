/*
 * @brief DAC example
 * This example show how to use the D/A Conversion in 2 modes: Polling and DMA
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

/** @defgroup EXAMPLES_PERIPH_17XX40XX_DAC LPC17xx/40xx DAC example
 * @ingroup EXAMPLES_PERIPH_17XX40XX
 * <b>Example description</b><br>
 * This example shows how to use DAC peripheral with 2 modes:
 * POLLING mode or DMA mode.<br>
 *
 * The DAC will be initialized with maximum current is 700uA. This allows a
 * maximum update rate of 1Mhz DAC updated values have range from 0 to 0x3FF.
 * AOUT ouput voltage will change from: Vss to VDD. Run and observe AOUT signal 
 * by oscilloscope.<br>
 *
 * The UART is used with a small menu to start and control the program.<br>
 *
 * <b>Special connection requirements</b><br>
 * - Embedded Artists' LPC1788 Developer's Kit:<br>
 * - Embedded Artists' LPC4088 Developer's Kit:<br>
 * Connect P0[26] (J3.28) to oscilloscope to observe the output signal.<br>
 * - LPCXpresso LPC1769:<br>
 * Need to connect with base board for using RS232/UART port.<br>
 * Connect P0[26] (J5.19) to oscilloscope to observe the output signal.<br>
 * J31, J32 on base board should be closed to hear the sound.
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

 #define DATA_SIZE 0x400
static const char WelcomeMenu[] =
	"Hello NXP Semiconductors \r\n"
	"DAC DEMO \r\n"
	"Press \'c\' to continue or \'x\' to quit\r\n";
static const char SelectMenu[] =
	"Press number 1-2 to choose DAC running mode:\r\n"
	"\t1: Polling Mode \r\n"
	"\t2: DMA Mode \r\n";
static const char DirectionMenu[] =
	"Press \'o\' or \'p\' to change sound frequency\r\n";

/* Work variables */
static volatile uint8_t channelTC, dmaChannelNum;
static volatile uint8_t DAC_Interrupt_Done_Flag, Interrupt_Continue_Flag;
static uint32_t DMAbuffer;

/* DAC sample rate request time */
#define DAC_TIMEOUT 0x3FF

/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/

/*****************************************************************************
 * Private functions
 ****************************************************************************/

/* DMA routine for DAC example */
static void App_DMA_Test(void)
{
	uint32_t tmp = 0;
	uint8_t freq_sound = 0xFF;
	uint8_t uart_buffer = 0;

	DEBUGOUT("DMA mode selected\r\n");
	DEBUGOUT(DirectionMenu);

	/* Initialize GPDMA controller */
	Chip_GPDMA_Init(LPC_GPDMA);

	/* Setup GPDMA interrupt */
	NVIC_DisableIRQ(DMA_IRQn);
	NVIC_SetPriority(DMA_IRQn, ((0x01 << 3) | 0x01));
	NVIC_EnableIRQ(DMA_IRQn);

	/* Get the free channel for DMA transfer */
	dmaChannelNum = Chip_DMA_GetFreeChannel(LPC_GPDMA, GPDMA_CONN_DAC);

	/* Output DAC value until get 'x' character */
	while (uart_buffer != 'x') {
		uart_buffer = DEBUGIN();
		if (uart_buffer == 'p') {
			freq_sound++;
		}
		else if (uart_buffer == 'o') {
			freq_sound--;
		}

		/* Start D/A conversion */
		tmp += (freq_sound % DATA_SIZE);
		if (tmp == (DATA_SIZE - 1)) {
			tmp = 0;
		}

		/* pre-format the data to DACR register */
		DMAbuffer = (uint32_t) (DAC_VALUE(tmp) | DAC_BIAS_EN);
		channelTC = 0;
		Chip_DMA_Transfer(LPC_GPDMA, dmaChannelNum,
						  (uint32_t) &DMAbuffer,
						  GPDMA_CONN_DAC,
						  GPDMA_TRANSFERTYPE_M2P_CONTROLLER_DMA,
						  1);

		/* Waiting for writing DAC value completed */
		while (channelTC == 0) {}
	}

	/* Disable interrupts, release DMA channel */
	Chip_DMA_Stop(LPC_GPDMA, dmaChannelNum);
	NVIC_DisableIRQ(DMA_IRQn);
}

/* Polling routine for DAC example */
static void App_Polling_Test(void)
{
	uint32_t tmp = 0;
	uint8_t freq_sound = 0xFF;
	uint8_t uart_buffer = 0;

	DEBUGOUT("Select Polling mode\r\n");
	DEBUGOUT(DirectionMenu);

	while (uart_buffer != 'x') {
		uart_buffer = DEBUGIN();
		if (uart_buffer == 'p') {
			freq_sound++;
		}
		else if (uart_buffer == 'o') {
			freq_sound--;
		}

		tmp += (freq_sound % DATA_SIZE);
		if (tmp == (DATA_SIZE - 1)) {
			tmp = 0;
		}
		Chip_DAC_UpdateValue(LPC_DAC, tmp);

		/* Wait for DAC (DMA) interrupt request */
		while (!(Chip_DAC_GetIntStatus(LPC_DAC))) {}
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
	if (Chip_DMA_Interrupt(LPC_GPDMA, dmaChannelNum) == SUCCESS) {
		channelTC++;
	}
	else {
		/* Error, do nothing */
	}
}

/**
 * @brief	Main routine for DAC example
 * @return	Nothing
 */
int main(void)
{
	uint8_t bufferUART;
	uint32_t dacClk;
	bool end_Flag = false;

	Board_Init();

	/* Setup DAC pins for board and common CHIP code */
	Chip_DAC_Init(LPC_DAC);

	/* Setup DAC timeout for polled and DMA modes to 0x3FF */
#if defined(CHIP_LPC175X_6X)
	/* 175x/6x devices have a DAC divider, set it to 1 */
	Chip_Clock_SetPCLKDiv(SYSCTL_PCLK_DAC, SYSCTL_CLKDIV_1);
#endif
	Chip_DAC_SetDMATimeOut(LPC_DAC, DAC_TIMEOUT);

	/* Compute and show estimated DAC request time */
#if defined(CHIP_LPC175X_6X)
	dacClk = Chip_Clock_GetPeripheralClockRate(SYSCTL_PCLK_DAC);
#else
	dacClk = Chip_Clock_GetPeripheralClockRate();
#endif
	DEBUGOUT("DAC base clock rate = %dHz, DAC request rate = %dHz\r\n",
		dacClk, (dacClk / DAC_TIMEOUT));

	/* Enable count and DMA support */
	Chip_DAC_ConfigDAConverterControl(LPC_DAC, DAC_CNT_ENA | DAC_DMA_ENA);

	while (!end_Flag) {
		DEBUGOUT(WelcomeMenu);
		while (!end_Flag) {
			bufferUART = 0xFF;
			bufferUART = DEBUGIN();
			if (bufferUART == 'c') {
				DEBUGOUT(SelectMenu);
				bufferUART = 0xFF;
				while (bufferUART == 0xFF) {
					bufferUART = DEBUGIN();
					if ((bufferUART != '1') && (bufferUART != '2') && (bufferUART != '3')) {
						bufferUART = 0xFF;
					}
				}
				switch (bufferUART) {
				case '1':		/* Polling Mode */
					App_Polling_Test();
					break;

				case '2':		/* DMA mode */
					App_DMA_Test();
					break;
				}
				break;
			}
			else if (bufferUART == 'x') {
				end_Flag = true;
				DEBUGOUT("DAC demo terminated!\r\n");
			}
		}
	}

	return 0;
}

/**
 * @}
 */
