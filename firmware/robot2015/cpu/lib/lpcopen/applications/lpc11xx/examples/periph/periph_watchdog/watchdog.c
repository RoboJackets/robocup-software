/*
 * @brief WWDT example
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

/** @defgroup EXAMPLES_PERIPH_11XX_WWDT LPC11xx Watchdog timer example
 * @ingroup EXAMPLES_PERIPH_11XX
 * <b>Example description</b><br>
 * This example shows how to use WWDT(Windowed Watchdog Timer)/WDT(Watchdog Timer)
 * to generate interrupts to manage the watchdog timer.<br>
 * If the watchdog timer isn't 'fed' within it's window time, it will
 * reset the device. The LED will toggle states on each watchdog feed event.
 * If the feed is disabled, the chip will reset.<br>
 *
 * The example works in one of 2 ways. For WWDT, the default way is to feed the
 * watchdog faster than it's warning interval. This is the normal operation of
 * the watchdog. The 2nd way of WWDT lets the watchdog generate a warning and
 * then feeds the watchdog timer on the warning (the LED will toggle on each
 * warning interval cycle). For WDT, the default way is to feed the watchdog
 * before time-out flag is set. The 2nd way lets the watchdog generate a time-out
 * interrupt (the LED will toggle on each time-out cycle).<br>
 * The 2nd way (both WWDT and WDT) can be enabled by commenting out the
 * DISABLE_WDT_TIMEOUT definition.<br>
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

/* Comment this define to let the watchdog timeout. In this case, the board
   will continuously drop via to the WDT warning. */
#define DISABLE_WDT_TIMEOUT

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
 * @brief	Watchdog Timer Interrupt Handler
 * @return	Nothing
 * @note	Handles watchdog timer warning and timeout events
 */
void WDT_IRQHandler(void)
{
	uint32_t wdtStatus = Chip_WWDT_GetStatus(LPC_WWDT);

#if defined(CHIP_LPC11CXX)
	if (wdtStatus & WWDT_WDMOD_WDTOF) {
		Board_LED_Toggle(0);
		while(Chip_WWDT_GetStatus(LPC_WWDT) & WWDT_WDMOD_WDTOF) {
			Chip_WWDT_ClearStatusFlag(LPC_WWDT, WWDT_WDMOD_WDTOF);
		}
		Chip_WWDT_Start(LPC_WWDT);	/* Needs restart */
	}
#else
	Board_LED_Toggle(0);

	/* The chip will reset before this happens, but if the WDT doesn't
	   have WWDT_WDMOD_WDRESET enabled, this will hit once */
	if (wdtStatus & WWDT_WDMOD_WDTOF) {
		/* A watchdog feed didn't occur prior to window timeout */
		Chip_WWDT_ClearStatusFlag(LPC_WWDT, WWDT_WDMOD_WDTOF);
		Chip_WWDT_Start(LPC_WWDT);	/* Needs restart */
	}

	/* Handle warning interrupt */
	if (wdtStatus & WWDT_WDMOD_WDINT) {
		/* A watchdog feed didn't occur prior to warning timeout */
		Chip_WWDT_ClearStatusFlag(LPC_WWDT, WWDT_WDMOD_WDINT);
		Chip_WWDT_Feed(LPC_WWDT);
	}
#endif
}

/**
 * @brief	SysTick Interrupt Handler
 * @return	Nothing
 * @note	Systick interrupt handler feeds WWDT
 */
void SysTick_Handler(void)
{
#if !defined(DISABLE_WDT_TIMEOUT)
	Chip_WWDT_Feed(LPC_WWDT);
#endif
}

/**
 * @brief	Main routine for WWDT example
 * @return	Nothing
 */
int main(void)
{
	uint32_t wdtFreq;

	/* Board Setup */
	Board_Init();

	/* Initialize WWDT (also enables WWDT clock) */
	Chip_WWDT_Init(LPC_WWDT);

	/* Prior to initializing the watchdog driver, the clocking for the
	   watchdog must be enabled. This example uses the watchdog oscillator
	   set at a 50KHz (1Mhz / 20) clock rate. */
	Chip_SYSCTL_PowerUp(SYSCTL_POWERDOWN_WDTOSC_PD);
	Chip_Clock_SetWDTOSC(WDTLFO_OSC_1_05, 20);

	/* The WDT divides the input frequency into it by 4 */
	wdtFreq = Chip_Clock_GetWDTOSCRate() / 4;

	/* LPC1102/4, LPC11XXLV, and LPC11CXX devices select the watchdog
	   clock source from the SYSCLK block, while LPC11AXX, LPC11EXX, and
	   LPC11UXX devices select the clock as part of the watchdog block. */
	/* Select watchdog oscillator for WDT clock source */
#if defined(CHIP_LPC110X) || defined(CHIP_LPC11XXLV) || defined(CHIP_LPC11CXX) || defined(CHIP_LPC11EXX)
	Chip_Clock_SetWDTClockSource(SYSCTL_WDTCLKSRC_WDTOSC, 1);
#else
	Chip_WWDT_SelClockSource(LPC_WWDT, WWDT_CLKSRC_WATCHDOG_WDOSC);
#endif

	Board_LED_Set(0, false);

	/* Set watchdog feed time constant to approximately 2s
	   Set watchdog warning time to 512 ticks after feed time constant
	   Set watchdog window time to 3s */
	Chip_WWDT_SetTimeOut(LPC_WWDT, wdtFreq * 2);
#if !defined(CHIP_LPC11CXX)
	Chip_WWDT_SetWarning(LPC_WWDT, 512);
	Chip_WWDT_SetWindow(LPC_WWDT, wdtFreq * 3);
#endif

#if !defined(CHIP_LPC11CXX)
	/* Configure WWDT to reset on timeout */
	Chip_WWDT_SetOption(LPC_WWDT, WWDT_WDMOD_WDRESET);
#endif

	/* Setup Systick to feed the watchdog timer. This needs to be done
	 * at a rate faster than the WDT warning. */
	SysTick_Config(Chip_Clock_GetSystemClockRate() / 50);

	/* Clear watchdog warning and timeout interrupts */
#if !defined(CHIP_LPC11CXX)	
	Chip_WWDT_ClearStatusFlag(LPC_WWDT, WWDT_WDMOD_WDTOF | WWDT_WDMOD_WDINT);
#else
	Chip_WWDT_ClearStatusFlag(LPC_WWDT, WWDT_WDMOD_WDTOF);
#endif	

	/* Clear and enable watchdog interrupt */
	NVIC_ClearPendingIRQ(WDT_IRQn);
	NVIC_EnableIRQ(WDT_IRQn);

	/* Start watchdog */
	Chip_WWDT_Start(LPC_WWDT);

	/* Idle while waiting */
	while (1) {
		__WFI();
	}
}

/**
 * @}
 */
