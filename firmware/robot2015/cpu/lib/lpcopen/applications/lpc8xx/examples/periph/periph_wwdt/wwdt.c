/*
 * @brief Windowed Watchdog Timer (WWDT) example
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

/** @defgroup EXAMPLES_PERIPH_8XX_WWDT LPC8xx Windowed Watchdog Timer example
 * @ingroup EXAMPLES_PERIPH_8XX
 * <b>Example description</b><br>
 * The WWDT example demonstrates the handling of WDT warning trigger interrupt
 * to enable "safe" operation.
 * The watchdog generates a warning WWDT interrupt and then feeds the WWDT
 * on the warning (the LED0 will toggle on each warning interval cycle).<br>
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
 * @brief	watchdog timer Interrupt Handler
 * @return	Nothing
 * @note	Handles watchdog timer warning and timeout events
 */
void WDT_IRQHandler(void)
{
	uint32_t wdtStatus = Chip_WWDT_GetStatus(LPC_WWDT);

	Board_LED_Toggle(0);

	/* The chip will reset before this happens, but if the WDT doesn't
	   have WWDT_WDMOD_WDRESET enabled, this will hit once */
	if (wdtStatus & WWDT_WDMOD_WDTOF) {
		/* A watchdog feed didn't occur prior to window timeout */
		Chip_WWDT_UnsetOption(LPC_WWDT, WWDT_WDMOD_WDEN);	/* Stop WDT */
		Chip_WWDT_ClearStatusFlag(LPC_WWDT, WWDT_WDMOD_WDTOF);
		Chip_WWDT_Start(LPC_WWDT);	/* Needs restart */
	}

	/* Handle warning interrupt */
	if (wdtStatus & WWDT_WDMOD_WDINT) {
		/* A watchdog feed didn't occur prior to warning timeout */
		Chip_WWDT_ClearStatusFlag(LPC_WWDT, WWDT_WDMOD_WDINT);
		Chip_WWDT_Feed(LPC_WWDT);
	}
}

/**
 * @brief	Application Main entry point
 * @return	Nothing (This will not return)
 */
int main(void)
{
	uint32_t wdtFreq;

	Board_Init();

	/* Freq = 0.6Mhz, divided by 64. WDT_OSC should be 9.375khz */
	Chip_Clock_SetWDTOSC(WDTLFO_OSC_0_60, 64);

	/* Enable the power to the WDT */
	Chip_SYSCTL_PowerUp(SYSCTL_SLPWAKE_WDTOSC_PD);

	/* The WDT divides the input frequency into it by 4 */
	wdtFreq = Chip_Clock_GetWDTOSCRate() / 4;

	Board_LED_Set(0, false);

	/* Initialize WWDT (also enables WWDT clock) */
	Chip_WWDT_Init(LPC_WWDT);

	/* Set watchdog feed time constant to approximately 2s
	   Set watchdog warning time to 512 ticks after feed time constant
	   Set watchdog window time to 3s */
	Chip_WWDT_SetTimeOut(LPC_WWDT, wdtFreq * 2);
	Chip_WWDT_SetWarning(LPC_WWDT, 512);
	Chip_WWDT_SetWindow(LPC_WWDT, wdtFreq * 3);

	/* Configure WWDT to reset on timeout */
	Chip_WWDT_SetOption(LPC_WWDT, WWDT_WDMOD_WDRESET);

	/* Clear watchdog warning and timeout interrupts */
	Chip_WWDT_ClearStatusFlag(LPC_WWDT, WWDT_WDMOD_WDTOF | WWDT_WDMOD_WDINT);

	/* Clear and enable watchdog interrupt */
	NVIC_ClearPendingIRQ(WDT_IRQn);
	NVIC_EnableIRQ(WDT_IRQn);

	/* Start watchdog */
	Chip_WWDT_Start(LPC_WWDT);

	while (1) {
		__WFI();
	}
	return 0;
}

/**
 * @}
 */
