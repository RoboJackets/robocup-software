/*
 * @brief State Configurable Timer (SCT) example
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

/** @defgroup EXAMPLES_PERIPH_8XX_SCT LPC8xx State Configurable Timer (SCT) example
 * @ingroup EXAMPLES_PERIPH_8XX
 * <b>Example description</b><br>
 * The SCT example demonstrates the match functionality using the
 * State Configurable Timer.<br>
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

#define TICKRATE_HZ (10)/* 10 ticks per second */

static volatile uint32_t ticks;

/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/

/*****************************************************************************
 * Private functions
 ****************************************************************************/

/*****************************************************************************
 * Public functions
 ****************************************************************************/

/**
 * @brief	Handle interrupt from SysTick timer
 * @return	Nothing
 */
void SysTick_Handler(void)
{
	++ticks;
	Board_LED_Toggle(1);
}

/**
 * @brief	Handle interrupt from State Configurable Timer
 * @return	Nothing
 */
void SCT_IRQHandler(void)
{
	if (ticks % 2) {
		Board_LED_Toggle(2);
	}
	else {
		Board_LED_Toggle(0);
	}

	/* Clear the Interrupt */
	Chip_SCT_ClearEventFlag(LPC_SCT, SCT_EVT_0);
}

/**
 * @brief	Application main program
 * @return	Nothing (This function will not return)
 */
int main(void)
{
	/* Generic Initialization */
	Board_Init();

	/* Enable SysTick Timer */
	if (SysTick_Config(SystemCoreClock / TICKRATE_HZ)) {
		while (1) {	/* error with configuring SysTick */
		}
	}

	/* Custom Initialization */
	Chip_SCT_Init(LPC_SCT);

	/* Configure the SCT as a 32bit counter using the bus clock */
	Chip_SCT_Config(LPC_SCT, SCT_CONFIG_32BIT_COUNTER | SCT_CONFIG_CLKMODE_BUSCLK);

	/* The match/capture REGMODE defaults to match mode */

	/* Set the match count for match register 0 */
	Chip_SCT_SetMatchCount(LPC_SCT, SCT_MATCH_0, SystemCoreClock / TICKRATE_HZ);

	/* Set the match reload value */
	Chip_SCT_SetMatchReload(LPC_SCT, SCT_MATCH_0, SystemCoreClock / TICKRATE_HZ);

	/* Enable an Interrupt on the Match Event */
	Chip_SCT_EventIntEnable(LPC_SCT, SCT_EVT_0);

	/* Enable the IRQ for the SCT */
	NVIC_EnableIRQ(SCT_IRQn);

	while (1) {
		Board_LED_Toggle(0);
		__WFI();
	}

	return 0;
}

/**
 * @}
 */
