/*
 * @brief RITimer example
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

/** @defgroup EXAMPLES_PERIPH_17XX40XX_RIT LPC17xx/40xx RITimer example
 * @ingroup EXAMPLES_PERIPH_17XX40XX
 * <b>Example description</b><br>
 * This example describes how to use the RIT driver as a periodic interrupt
 * source. The RIT time interval is configured to 1s. On each RITimer interrupt,
 * the LED state will toggle.<br>
 *
 * <b>Special connection requirements</b><br>
 * There are no special connection requirements for this example.<br>
 *
 * <b>Build procedures:</b><br>
 * @ref LPCOPEN_17XX40XX_BUILDPROCS_KEIL<br>
 * @ref LPCOPEN_17XX40XX_BUILDPROCS_IAR<br>
 * @ref LPCOPEN_17XX40XX_BUILDPROCS_XPRESSO<br>
 *
 * <b>Supported boards and board setup:</b><br>
 * @ref LPCOPEN_17XX40XX_BOARD_XPRESSO_1769<br>
 *
 * <b>Submitting LPCOpen issues:</b><br>
 * @ref LPCOPEN_COMMUNITY
 * @{
 */

/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/

/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/

/*****************************************************************************
 * Private functions
 ****************************************************************************/

#define TIME_INTERVAL   (1000)
static volatile bool On;

/*****************************************************************************
 * Public functions
 ****************************************************************************/

/**
 * @brief	RIT interrupt handler
 * @return	Nothing
 */
void RIT_IRQHandler(void)
{
	/* Clearn interrupt */
	Chip_RIT_ClearInt(LPC_RITIMER);

	/* Toggle LED */
	On = (bool) !On;
	Board_LED_Set(0, On);
}

/**
 * @brief	Main entry point
 * @return	Nothing
 */
int main(void)
{
	Board_Init();

	On = true;
	Board_LED_Set(0, On);

	/* Initialize RITimer */
	Chip_RIT_Init(LPC_RITIMER);

	/* Configure RIT for a 1s interrupt tick rate */
	Chip_RIT_SetTimerInterval(LPC_RITIMER, TIME_INTERVAL);

	NVIC_EnableIRQ(RITIMER_IRQn);

	/* LED is toggled in interrupt handler */
	while (1) {}
}

/**
 * @}
 */
