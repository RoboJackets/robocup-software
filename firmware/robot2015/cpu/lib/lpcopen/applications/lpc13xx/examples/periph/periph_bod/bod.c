/*
 * @brief Brown-out detector example
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

/** @defgroup EXAMPLES_PERIPH_13XX_BOD LPC13xx Brown-out detector example
 * @ingroup EXAMPLES_PERIPH_13XX
 * <b>Example description</b><br>
 * The brown-out example shows how to use the brown-out detector (BOD)
 * on the LPC11XX.The BOD is setup to generate a BOD interrupt when
 * power is lost. The interrupt will attempt to toggle the LED on as
 * power is lost.<br>
 *
 * To use this example, build and program it and then run it on the board.
 * Power off the board by removing the power (USB) cable. As the power is
 * declining on the board, the LED will toggle on (quickly) and then turn
 * off as power is lost.<br>
 *
 * <b>Special connection requirements</b><br>
 * There are no special connection requirements for this example.<br>
 *
 * <b>Build procedures:</b><br>
 * @ref LPCOPEN_13XX_BUILDPROCS_XPRESSO<br>
 * @ref LPCOPEN_13XX_BUILDPROCS_KEIL<br>
 * @ref LPCOPEN_13XX_BUILDPROCS_IAR<br>
 *
 * <b>Supported boards and board setup:</b><br>
 * @ref LPCOPEN_13XX_BOARD_XPRESSO_1347<br>
 *
 * <b>Submitting LPCOpen issues:</b><br>
 * @ref LPCOPEN_COMMUNITY
 * @{
 */

/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/

static volatile uint8_t isPORReset = 0;

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
 * @brief	Handle interrupt from Brown-out detector
 * @return	Nothing
 */
void BOD_IRQHandler(void)
{
	/* Toggle LED on if brown-out detector trips */
	Board_LED_Set(0, true);

	/* Disable interrupt so it won't keep firing */
	NVIC_DisableIRQ(BOD_IRQn);
}

/**
 * @brief	main routine for brown-out example
 * @return	Function should not exit.
 */
int main(void)
{
	uint32_t sysResetStatus;
	volatile int loop = 1;

	/* Generic Initialization */
	Board_Init();

	Board_LED_Set(0, false);

	/* Set brown-out interrupt level with reset  */
	Chip_SYSCTL_SetBODLevels(SYSCTL_BODRSTLVL_2_06V, SYSCTL_BODINTVAL_2_80V);
	Chip_SYSCTL_EnableBODReset();

	/* Enable BOD interrupt */
	NVIC_ClearPendingIRQ(BOD_IRQn);
	NVIC_EnableIRQ(BOD_IRQn);

	/* If the board was reset due to a BOD event, the reset can be
	   detected here. If the board was completely powered off, the BOD
	   reset event won't be active. */
	sysResetStatus = Chip_SYSCTL_GetSystemRSTStatus();
	if ((sysResetStatus & SYSCTL_RST_BOD) == 0) {
		/* Board was reset via a normal power-on reset event */
		sysResetStatus |= SYSCTL_RST_POR;
	}
	else {
		/* Board was reset via a BOD event */
		sysResetStatus |= SYSCTL_RST_BOD;
	}

	/* Clear reset status */
	Chip_SYSCTL_ClearSystemRSTStatus(sysResetStatus);

	/* Wait forever */
	while (loop) {
		__WFI();
	}

	return 0;
}

/**
 * @}
 */
