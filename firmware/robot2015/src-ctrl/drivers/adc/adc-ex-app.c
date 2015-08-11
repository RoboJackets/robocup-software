/*
 * @brief ADC example
 * This example show how to  the ADC in 3 mode : Polling, Interrupt and DMA
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

#include "../../../../software/lpc_core/lpc_board/boards_17xx_40xx/board.h/nxp_xpresso_1769/board.h"

/** @defgroup EXAMPLES_PERIPH_17XX40XX_ADC LPC17xx/40xx ADC example
 * @ingroup EXAMPLES_PERIPH_17XX40XX
 * <b>Example description</b><br>
 * This example describes how to use ADC conversion in POLLING mode, INTERRUPT mode
 * and DMA mode.<br>
 *
 * For LPC1788 and LPC4088, the ADC conversion rate is 400KHz. A/D converter requires 31 clocks
 * for a fully accurate conversion. Therefore, ADC clock = 400KHz * (number of clocks for 1 measure = 31 clocks).
 * Note that maximum ADC clock input is 12.4MHz.<br>
 *
 * For LPC1769, the maximum ADC clock input is 13MHz. 65 ADC converter's clocks are required
 * for a full conversion. The conversion rate is 200KHz.<br>
 *
 * The ADC value can be read in POLLING mode, INTERRUPT mode or DMA mode. Converted ADC values
 * displayed periodically via the UART. Turn potentiometer to change ADC signal input.<br>
 *
 * Setting up the demo requires connecting a UART cable between the board and a host PC. The
 * terminal program on the host PC should be setup for 115K8N1. Press the appropriate key via
 * the menu to change the ADC conversion mode.<br>
 *
 * <b>Special connection requirements</b><br>
 * - Embedded Artists' LPC1788 Developer's Kit:<br>
 * - Embedded Artists' LPC4088 Developer's Kit:<br>
 * There are no special connection requirements for this example.<br>
 * - LPCXpresso LPC1769:<br>
 * Need to connect with base board for using RS232/UART port.<br>
 * J27 on the base board must be closed.
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
#ifdef BOARD_NXP_XPRESSO_1769
#define _ADC_CHANNLE ADC_CH0
#else
#define _ADC_CHANNLE ADC_CH2
#endif
#define _LPC_ADC_ID LPC_ADC
#define _LPC_ADC_IRQ ADC_IRQn
#define _GPDMA_CONN_ADC GPDMA_CONN_ADC
static char WelcomeMenu[] = "\r\nHello NXP Semiconductors \r\n"
							"ADC DEMO \r\n"
#if defined(CHIP_LPC175X_6X)							
							"Sample rate : 200kHz \r\n"
#else
							"Sample rate : 400kHz \r\n"
#endif
							"Press \'c\' to continue or \'x\' to quit\r\n"
							"Press \'o\' or \'p\' to set Sample rate\r\n"
							"Press \'b\' to ENABLE or DISABLE Burst Mode\r\n";
static char SelectMenu[] = "\r\nPress number 1-3 to choose ADC running mode:\r\n"
						   "\t1: Polling Mode \r\n"
						   "\t2: Interrupt Mode \r\n"
						   "\t3: DMA Mode \r\n";

static ADC_Clock_Setup_T ADCSetup;
static volatile uint8_t Burst_Mode_Flag = 0, Interrupt_Continue_Flag;
static volatile uint8_t ADC_Interrupt_Done_Flag, channelTC, dmaChannelNum;
uint32_t DMAbuffer;
/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/

/*****************************************************************************
 * Private functions
 ****************************************************************************/

/* Print ADC value and delay */
static void App_print_ADC_value(uint16_t data)
{
	volatile uint32_t j;
	j = 5000000;
	DEBUGOUT("ADC value is : 0x%04x\r\n", data);
	/* Delay */
	while (j--) {}
}

/* DMA routine for ADC example */
static void App_DMA_Test(void)
{
	uint16_t dataADC;

	/* Initialize GPDMA controller */
	Chip_GPDMA_Init(LPC_GPDMA);
	/* Setting GPDMA interrupt */
	NVIC_DisableIRQ(DMA_IRQn);
	NVIC_SetPriority(DMA_IRQn, ((0x01 << 3) | 0x01));
	NVIC_EnableIRQ(DMA_IRQn);
	/* Setting ADC interrupt, ADC Interrupt must be disable in DMA mode */
	NVIC_DisableIRQ(_LPC_ADC_IRQ);
	Chip_ADC_Channel_Int_Cmd(_LPC_ADC_ID, _ADC_CHANNLE, ENABLE);
	/* Get the free channel for DMA transfer */
	dmaChannelNum = Chip_DMA_GetFreeChannel(LPC_GPDMA, _GPDMA_CONN_ADC);
	/* Enable burst mode if any, the AD converter does repeated conversions
	   at the rate selected by the CLKS field in burst mode automatically */
	if (Burst_Mode_Flag) {
		Chip_ADC_Burst_Cmd(_LPC_ADC_ID, ENABLE);
	}
	/* Get  adc value until get 'x' character */
	while (DEBUGIN() != 'x') {
		/* Start A/D conversion if not using burst mode */
		if (!Burst_Mode_Flag) {
			Chip_ADC_Set_StartMode(_LPC_ADC_ID, ADC_START_NOW, ADC_TRIGGERMODE_RISING);
		}
		channelTC = 0;
		Chip_DMA_Transfer(LPC_GPDMA, dmaChannelNum,
						  _GPDMA_CONN_ADC,
						  (uint32_t) &DMAbuffer,
						  GPDMA_TRANSFERTYPE_P2M_CONTROLLER_DMA,
						  1);

		/* Waiting for reading ADC value completed */
		while (channelTC == 0) {}

		/* Get the ADC value fron Data register*/
		dataADC = ADC_DR_RESULT(DMAbuffer);
		App_print_ADC_value(dataADC);
	}
	/* Disable interrupts, release DMA channel */
	Chip_DMA_Stop(LPC_GPDMA, dmaChannelNum);
	NVIC_DisableIRQ(DMA_IRQn);
	/* Disable burst mode if any */
	if (Burst_Mode_Flag) {
		Chip_ADC_Burst_Cmd(_LPC_ADC_ID, DISABLE);
	}
}

/* Interrupt routine for ADC example */
static void App_Interrupt_Test(void)
{
	/* Enable ADC Interrupt */
	NVIC_EnableIRQ(_LPC_ADC_IRQ);
	Chip_ADC_Channel_Int_Cmd(_LPC_ADC_ID, _ADC_CHANNLE, ENABLE);
	/* Enable burst mode if any, the AD converter does repeated conversions
	   at the rate selected by the CLKS field in burst mode automatically */
	if (Burst_Mode_Flag) {
		Chip_ADC_Burst_Cmd(_LPC_ADC_ID, ENABLE);
	}
	Interrupt_Continue_Flag = 1;
	ADC_Interrupt_Done_Flag = 1;
	while (Interrupt_Continue_Flag) {
		if (!Burst_Mode_Flag && ADC_Interrupt_Done_Flag) {
			ADC_Interrupt_Done_Flag = 0;
			Chip_ADC_Set_StartMode(_LPC_ADC_ID, ADC_START_NOW, ADC_TRIGGERMODE_RISING);
		}
	}
	/* Disable burst mode if any */
	if (Burst_Mode_Flag) {
		Chip_ADC_Burst_Cmd(_LPC_ADC_ID, DISABLE);
	}
	/* Disable ADC interrupt */
	NVIC_DisableIRQ(_LPC_ADC_IRQ);
}

/* Polling routine for ADC example */
static void App_Polling_Test(void)
{
	uint16_t dataADC;

	/* Select using burst mode or not */
	if (Burst_Mode_Flag) {
		Chip_ADC_Burst_Cmd(_LPC_ADC_ID, ENABLE);
	}
	else {
		Chip_ADC_Burst_Cmd(_LPC_ADC_ID, DISABLE);
	}

	/* Get  adc value until get 'x' character */
	while (DEBUGIN() != 'x') {
		/* Start A/D conversion if not using burst mode */
		if (!Burst_Mode_Flag) {
			Chip_ADC_Set_StartMode(_LPC_ADC_ID, ADC_START_NOW, ADC_TRIGGERMODE_RISING);
		}
		/* Waiting for A/D conversion complete */
		while (Chip_ADC_Read_Status(_LPC_ADC_ID, _ADC_CHANNLE, ADC_DR_DONE_STAT) != SET) {}
		/* Read ADC value */
		Chip_ADC_Read_Value(_LPC_ADC_ID, _ADC_CHANNLE, &dataADC);
		/* Print ADC value */
		App_print_ADC_value(dataADC);
	}

	/* Disable burst mode, if any */
	if (Burst_Mode_Flag) {
		Chip_ADC_Burst_Cmd(_LPC_ADC_ID, DISABLE);
	}
}

/*****************************************************************************
 * Public functions
 ****************************************************************************/

/**
 * @brief	ADC0 interrupt handler sub-routine
 * @return	Nothing
 */
void ADC_IRQHandler(void)
{
	uint16_t dataADC;
	/* Interrupt mode: Call the stream interrupt handler */
	NVIC_DisableIRQ(_LPC_ADC_IRQ);
	Chip_ADC_Channel_Int_Cmd(_LPC_ADC_ID, _ADC_CHANNLE, DISABLE);
	Chip_ADC_Read_Value(_LPC_ADC_ID, _ADC_CHANNLE, &dataADC);
	ADC_Interrupt_Done_Flag = 1;
	App_print_ADC_value(dataADC);
	for(int i=0; i<ADC_NUM_CHANNELS; i++)
		NVIC_EnableIRQ(_LPC_ADC_IRQ);
		Chip_ADC_Channel_Int_Cmd(_LPC_ADC_ID, _ADC_CHANNLE, ENABLE);
	}
	else {Interrupt_Continue_Flag = 0; }
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
 * @brief	Main routine for ADC example
 * @return	Nothing
 */
int main(void)
{
	bool end_Flag = false;
	uint32_t _bitRate = ADC_MAX_SAMPLE_RATE;
	uint8_t bufferUART;

	Board_Init();

	/*	Chip_IOCON_PinMux(0, 25, IOCON_ADMODE_EN, IOCON_FUNC1); */
	/*ADC Init */
	Chip_ADC_Init(_LPC_ADC_ID, &ADCSetup);
	Chip_ADC_Channel_Enable_Cmd(_LPC_ADC_ID, _ADC_CHANNLE, ENABLE);

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
				DEBUGOUT("\r\nADC demo terminated!");
			}
			else if (bufferUART == 'o') {
				_bitRate -= _bitRate > 0 ? 1000 : 0;
				Chip_ADC_Set_SampleRate(_LPC_ADC_ID, &ADCSetup, _bitRate);
				DEBUGOUT("Rate : %d Sample/s\r\n", _bitRate);
			}
			else if (bufferUART == 'p') {
				_bitRate += _bitRate < 400000 ? 1000 : 0;
				Chip_ADC_Set_SampleRate(_LPC_ADC_ID, &ADCSetup, _bitRate);
				DEBUGOUT("Rate : %d Sample/s\r\n", _bitRate);
			}
			else if (bufferUART == 'b') {
				Burst_Mode_Flag = !Burst_Mode_Flag;
				ADCSetup.burstMode = Burst_Mode_Flag;
				Chip_ADC_Set_SampleRate(_LPC_ADC_ID, &ADCSetup, _bitRate);
				if (Burst_Mode_Flag) {
					DEBUGOUT("Burst Mode ENABLED\r\n");
				}
				else {
					DEBUGOUT("Burst Mode DISABLED\r\n");
				}
			}
		}
	}
	return 0;
}

/**
 * @}
 */
