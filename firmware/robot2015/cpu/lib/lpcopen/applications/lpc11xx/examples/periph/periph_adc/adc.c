/*
 * @brief Analog to digital converter example
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
#include <stdio.h>

/** @defgroup EXAMPLES_PERIPH_11XX_ADC LPC11xx ADC example
 * @ingroup EXAMPLES_PERIPH_11XX
 * <b>Example description</b><br>
 * The ADC example shows how to use the A/D converter
 * on the LPC11XX. The example is setup to get the ADC values
 * periodically and print it out via UART.<br>
 *
 * To use this example, build and program it and then run it
 * on the LPCXpresso board with LPCXpresso Base Board rev A.
 * Turn potentiometer on Base board to change ADC signal input.Converted ADC values
 * is displayed periodically via the UART.<br>
 *
 * <b>Special connection requirements</b><br>
 * Connect the Jumper J27 on the LPCXpresso Base Board rev A. Connect the
 * UART to display ADC value on  your computer<br>
 *
 * <b>Build procedures:</b><br>
 * @ref LPCOPEN_11XX_BUILDPROCS_XPRESSO<br>
 * @ref LPCOPEN_11XX_BUILDPROCS_KEIL<br>
 * @ref LPCOPEN_11XX_BUILDPROCS_IAR<br>
 *
 * <b>Supported boards and board setup:</b><br>
 * @ref LPCOPEN_11XX_BOARD_XPRESSO_11U14<br>
 * @ref LPCOPEN_11XX_BOARD_XPRESSO_11C24<br>
 *
 * <b>Submitting LPCOpen issues:</b><br>
 * @ref LPCOPEN_COMMUNITY
 * @{
 */

/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/

#define _LPC_ADC_ID LPC_ADC
static ADC_Clock_Setup_T ADCSetup;

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
	char out_str[5];
	for (j = 4; j != 0; j--) {
		out_str[j - 1] = (data % 10) + 48;
		data /= 10;
	}
	out_str[4] = '\0';

	DEBUGSTR("\r\nADC value is ");
	DEBUGSTR(out_str);

	/* Delay */
	j = 500000;
	while (j--) {}
}

/* Simple ADC resr and display loop */
static void App_Polling_Test(void)
{
	uint16_t dataADC;

	while (1) {
		/* Start A/D conversion */
		Chip_ADC_Set_StartMode(_LPC_ADC_ID, ADC_START_NOW, ADC_TRIGGERMODE_RISING);

		/* Waiting for A/D conversion complete */
		while (Chip_ADC_Read_Status(_LPC_ADC_ID, ADC_CH0, ADC_DR_DONE_STAT) != SET) {}

		/* Read ADC value */
		Chip_ADC_Read_Value(_LPC_ADC_ID, ADC_CH0, &dataADC);

		/* Print ADC value */
		App_print_ADC_value(dataADC);
	}
}

/*****************************************************************************
 * Public functions
 ****************************************************************************/

/**
 * @brief	main routine for ADC example
 * @return	Function should not exit.
 */
int main(void)
{
	Board_Init();
	Board_ADC_Init();
	DEBUGSTR("ADC Demo\r\n");

	/*ADC Init */
	Chip_ADC_Init(_LPC_ADC_ID, &ADCSetup);
	Chip_ADC_Channel_Enable_Cmd(_LPC_ADC_ID, ADC_CH0, ENABLE);
	App_Polling_Test();

	/* Should not run to here */
	return 0;
}

/**
 * @}
 */
