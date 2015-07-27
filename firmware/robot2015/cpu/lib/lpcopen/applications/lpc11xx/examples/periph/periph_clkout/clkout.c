/*
 * @brief CLKOUT example
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

/** @defgroup EXAMPLES_PERIPH_11XX_CLKOUT LPC11xx Clock output example
 * @ingroup EXAMPLES_PERIPH_11XX
 * <b>Example description</b><br>
 * This example shows how to use the SYSCTL driver to driver a specific
 * clock source on the CLKOUT pin. To use this example, you'll need to
 * connect an oscilloscope to the CLKOUT pin on your board. See the
 * comments in the code for mapped pins for supported boards.<br>
 *
 * <b>Special connection requirements</b><br>
 * There are no special connection requirements for this example.<br>
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

static volatile uint32_t ticks100;

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
	ticks100++;
	if ((ticks100 % 100) == 0) {
		Board_LED_Toggle(0);
	}
}

/**
 * @brief	main routine for CLKOUT example
 * @return	Always returns 0.
 */
int main(void)
{
	CHIP_SYSCTL_CLKOUTSRC_T clkoutClks;

	Board_Init();

	/* Enable and setup SysTick Timer at a 100Hz rate */
	SysTick_Config(SystemCoreClock / 100);

	/* Enable the power to the WDT */
	Chip_SYSCTL_PowerUp(SYSCTL_POWERDOWN_WDTOSC_PD);

	/*This example uses the watchdog oscillator set at a 50KHz (1Mhz / 20) clock rate. */
	Chip_Clock_SetWDTOSC(WDTLFO_OSC_1_05, 20);

#if defined(CHIP_LPC11UXX) || defined(CHIP_LPC11EXX) || defined(CHIP_LPC11AXX)
	/* LPC11U14 Xpresso board has CLKOUT on pin PIO0_1 on J6-38 */
	Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 1, (IOCON_FUNC1 | IOCON_MODE_INACT));
#else
	/* LPC11C24 Xpresso board has CLKOUT on pin PIO0_1 on J6-38 */
	Chip_IOCON_PinMuxSet(LPC_IOCON, IOCON_PIO0_1, (IOCON_FUNC1 | IOCON_MODE_INACT));
#endif

	/* Cycle through all clock sources for the CLKOUT pin */
	for (clkoutClks = SYSCTL_CLKOUTSRC_IRC;
		 clkoutClks <= SYSCTL_CLKOUTSRC_MAINSYSCLK; clkoutClks++) {
		ticks100 = 0;

		/* Setup CLKOUT pin for specific clock with a divider of 1 */
		Chip_Clock_SetCLKOUTSource(clkoutClks, 1);

		/* Wait 5 seconds */
		while (ticks100 < 500) {
			__WFI();
		}
	}

	/* Disable CLKOUT pin by setting divider to 0 */
	Chip_Clock_SetCLKOUTSource(SYSCTL_CLKOUTSRC_IRC, 0);

	return 0;
}

/**
 * @}
 */
