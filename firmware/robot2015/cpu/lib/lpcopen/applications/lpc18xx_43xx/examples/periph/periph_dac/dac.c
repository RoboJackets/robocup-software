/*
 * @brief DAC example
 * This example show how to use the D/A Conversion in 3 modes: Polling, Interrupt and DMA
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

/** @defgroup EXAMPLES_PERIPH_18XX43XX_DAC LPC18xx/43xx DAC example
 * @ingroup EXAMPLES_PERIPH_18XX43XX
 * <b>Example description</b><br>
 * This example shows how to use DAC peripheral with 3 modes: POLLING mode,
 * INTERRUPT mode or DMA mode.<br>
 *
 * DAC will be initialized with maximum current is 700uA. This allows a
 * maximum update rate of 1Mhz DAC updated values have range from 0 to 0x3FF.
 * So AOUT output voltage will change from: Vss to VDD. This example
 * configures pin P4_4 as analog function for DAC output through ENAIO2
 * register. Run and observe AOUT signal by oscilloscope.<br>
 *
 * Use UART to monitor this demo.<br>
 *
 * <b>Special connection requirements</b><br>
 *  - Hitex LPC1850EVA-A4-2 and LPC4350EVA-A4-2 boards (AOUT: X16.4 (X16 connector - pin LCDVD1))<br>
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
 *
 * <b>Submitting LPCOpen issues:</b><br>
 * @ref LPCOPEN_COMMUNITY
 * @{
 */

/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/
#define DATA_SIZE 0x400
static char WelcomeMenu[] = "\r\nHello NXP Semiconductors \r\n"
							"DAC DEMO \r\n"
							"Press \'c\' to continue or \'x\' to quit\r\n";
static char SelectMenu[] = "\r\nPress number 1-3 to choose DAC running mode:\r\n"
						   "\t1: Polling Mode \r\n"
						   "\t2: Interrupt Mode \r\n"
						   "\t3: DMA Mode \r\n";
static volatile uint8_t channelTC, dmaChannelNum;
static volatile uint8_t DAC_Interrupt_Done_Flag, Interrupt_Continue_Flag;
uint32_t DMAbuffer;
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
	volatile uint32_t i = 0;

	/* Initialize GPDMA controller */
	Chip_GPDMA_Init(LPC_GPDMA);
	/* Setting GPDMA interrupt */
	NVIC_DisableIRQ(DMA_IRQn);
	NVIC_SetPriority(DMA_IRQn, ((0x01 << 3) | 0x01));
	NVIC_EnableIRQ(DMA_IRQn);
	/* Get the free channel for DMA transfer */
	dmaChannelNum = Chip_DMA_GetFreeChannel(LPC_GPDMA, GPDMA_CONN_DAC);

	/* Output DAC value until get 'x' character */
	while (DEBUGIN() != 'x') {
		/* Start D/A conversion */
		tmp++;
		if (tmp == (DATA_SIZE - 1)) {
			tmp = 0;
		}
		/* pre-format the data to DACR register */
		DMAbuffer = (uint32_t) (DAC_VALUE(tmp) | DAC_BIAS_EN);
		for (i = 0; i < 0x10000; i++) ;

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

/* Interrupt routine for DAC example */
static void App_Interrupt_Test(void)
{
	uint32_t tmp = 0;
	volatile uint32_t i = 0;

	NVIC_DisableIRQ(DAC_IRQn);
	NVIC_SetPriority(DAC_IRQn, ((0x01 << 2) | 0x01));

	Interrupt_Continue_Flag = 1;
	DAC_Interrupt_Done_Flag = 1;
	while (Interrupt_Continue_Flag) {
		tmp++;
		if (tmp == (DATA_SIZE - 1)) {
			tmp = 0;
		}
		Chip_DAC_UpdateValue(LPC_DAC, tmp);
		if (DAC_Interrupt_Done_Flag) {
			DAC_Interrupt_Done_Flag = 0;
			NVIC_EnableIRQ(DAC_IRQn);
		}
		for (i = 0; i < 0x10000; i++) ;
	}

	/* Disable DAC interrupt */
	NVIC_DisableIRQ(DAC_IRQn);
}

/* Polling routine for DAC example */
static void App_Polling_Test(void)
{
	uint32_t tmp = 0;
	volatile uint32_t i = 0;

	while (DEBUGIN() != 'x') {
		tmp++;
		if (tmp == (DATA_SIZE - 1)) {
			tmp = 0;
		}
		Chip_DAC_UpdateValue(LPC_DAC, tmp);

		while (!(Chip_DAC_GetIntStatus(LPC_DAC))) {}

		for (i = 0; i < 0x10000; i++) ;
	}
}

/*****************************************************************************
 * Public functions
 ****************************************************************************/

/**
 * @brief	DAC interrupt handler sub-routine
 * @return	Nothing
 */
void DAC_IRQHandler(void)
{
	NVIC_DisableIRQ(DAC_IRQn);
	DAC_Interrupt_Done_Flag = 1;
	if (DEBUGIN() == 'x') {
		Interrupt_Continue_Flag = 0;
	}
}

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
		/* Process error here */
	}
}

/**
 * @brief	Main routine for DAC example
 * @return	Nothing
 */
int main(void)
{
	bool end_Flag = false;
	uint8_t bufferUART;

	Board_Init();
	Board_DAC_Init(LPC_DAC);
	/* DAC Init */
	Chip_DAC_Init(LPC_DAC);
	/* set time out for DAC*/
	Chip_DAC_SetDMATimeOut(LPC_DAC, 0xFFFF);
	Chip_DAC_ConfigDAConverterControl(LPC_DAC, (DAC_CNT_ENA | DAC_DMA_ENA));

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

				case '2':		/* Interrupt Mode */
					App_Interrupt_Test();
					break;

				case '3':		/* DMA mode */
					App_DMA_Test();
					break;
				}
				break;
			}
			else if (bufferUART == 'x') {
				end_Flag = true;
				DEBUGOUT("\r\nDAC demo terminated!");
			}
		}
	}
	return 0;
}

/**
 * @}
 */
