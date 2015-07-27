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

/** @defgroup EXAMPLES_PERIPH_8XX_ACMP LPC8xx Analog Comparator example
 * @ingroup EXAMPLES_PERIPH_8XX
 * <b>Example description</b><br>
 * The ACMP example demonstrates using the analog comparator.<br>
 *
 * This example configures the positive voltage input as the voltage
 * found on PIN0.0, which is trimmed through the potentiometer.
 * The output of the comparator is brought out to PIN0.15 and
 * the "operating voltage" can be measured at PIN0.0.
 * Adjust the POT up and down to adjust the voltage into the analog
 * comparator. When the voltage crosses the negative voltage input,
 * a CMP IRQ is fired. Based on which side of the voltage is in
 * reference to the bandgap, the LED state will change.<br>
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

/*****************************************************************************
 * Public functions
 ****************************************************************************/

/**
 * @brief	Analog comparator interrupt handler sub-routine
 * @return	Nothing
 */
void CMP_IRQHandler(void)
{
	/* Clear the interrupt */
	Chip_ACMP_EdgeClear(LPC_CMP);

	__SEV();
}

/**
 * @brief	Setup ACMP example
 * @return	Nothing
 */
void HW_Setup(void)
{
	/* initialize the board */
	Board_Init();

	/* initialize the ACMP */
	Chip_ACMP_Init(LPC_CMP);

	/* Enable the clock to the Switch Matrix */
	Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_SWM);

	/* Configure the SWM for ACMP_I1 as the input for the Analog Comparator. */
	Chip_SWM_FixedPinEnable(ACMP_I1, ENABLE);

	/* Set the LPC8XX_ACMP_O_PINx (in sys_config.h) pin direction as an output. */
	Chip_GPIO_SetDir(LPC_GPIO_PORT, 0, LPC8XX_ACMP_O_PINx, 1);

	/* Configure the SWM for PIO15 as the output for the comparator. */
	Chip_SWM_MovablePinAssign(SWM_ACMP_O_O, PIO15);

	/* Positive and negative references, both edges, no hysteresis */
	Chip_ACMP_SetupAMCPRefs(LPC_CMP, ACMP_EDGESEL_BOTH, ACMP_POSIN_ACMP_I1, ACMP_NEGIN_INT_REF, ACMP_HYS_NONE);

	/* Disable the clock to the Switch Matrix to save power */
	Chip_Clock_DisablePeriphClock(SYSCTL_CLOCK_SWM);

	/* Enable the Interrupt for the compare output */
	NVIC_EnableIRQ(CMP_IRQn);
}

/**
 * @brief	Main UART program body
 * @return	Always returns -1
 */
int main(void)
{
	/* Board and hardware setup */
	HW_Setup();

	while (1) {
		/* Enter low power mode until interrupt */
		__WFE();

		if (Chip_ACMP_GetCompStatus(LPC_CMP) & ACMP_COMPSTAT_BIT) {
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
