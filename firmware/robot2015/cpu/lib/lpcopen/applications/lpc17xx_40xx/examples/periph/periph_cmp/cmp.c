/*
 * @brief Analog Comparator example.
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

/** @defgroup EXAMPLES_PERIPH_17XX40XX_CMP LPC17xx/40xx Comparator example
 * @ingroup EXAMPLES_PERIPH_17XX40XX
 * <b>Example description</b><br>
 * The CMP example demonstrates using the analog comparator.<br>
 *
 * This example configures the positive voltage input as the voltage
 * found on PIN0.25, which is trimmed through the potentiometer.
 * Adjust the POT up and down to adjust the voltage into the analog
 * comparator. When the voltage crosses the negative voltage input,
 * a CMP IRQ is fired. Based on which side of the voltage is in
 * reference to the bandgap, the LED state will change.<br>
 *
 * <b>Special connection requirements</b><br>
 * Connect P0.25 (J5.24) to P1.7 (J5.29) to input the comparator 0<br>
 *
 * <b>Build procedures:</b><br>
 * @ref LPCOPEN_17XX40XX_BUILDPROCS_KEIL<br>
 * @ref LPCOPEN_17XX40XX_BUILDPROCS_IAR<br>
 * @ref LPCOPEN_17XX40XX_BUILDPROCS_XPRESSO<br>
 *
 * <b>Supported boards and board setup:</b><br>
 * @ref LPCOPEN_17XX40XX_BOARD_EA4088<br>
 *
 * <b>Submitting LPCOpen issues:</b><br>
 * @ref LPCOPEN_COMMUNITY
 * @{
 */

/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/

#define CMP_ID      1

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
 * @brief	Main UART program body
 * @return	Doesn't return
 */
int main(void)
{
	/* initialize the board */
	Board_Init();

	Board_CMP_Init();

	/* initialize the CMP */
	Chip_CMP_Init();

	/* Power-up */
	Chip_CMP_EnableCurrentSrc(CMP_ENCTRL_ENABLE);
	Chip_CMP_EnableBandGap(CMP_ENCTRL_ENABLE);
	Chip_CMP_Enable(CMP_ID, CMP_ENCTRL_ENABLE);
	
	/* Positive and negative references, both edges, no hysteresis */
	Chip_CMP_SetPosVoltRef(CMP_ID, CMP_INPUT_CMPx_IN0);
	Chip_CMP_SetNegVoltRef(CMP_ID, CMP_INPUT_INTERNAL_09VBG);
	Chip_CMP_SetHysteresis(CMP_ID, CMP_HYS_NONE);
	
	while (1) {
		if (Chip_CMP_GetCmpStatus(CMP_ID)) {
			Board_LED_Set(0, false);
		}
		else {
			Board_LED_Set(0, true);
		}
	}

	return 0;
}

/**
 * @}
 */
