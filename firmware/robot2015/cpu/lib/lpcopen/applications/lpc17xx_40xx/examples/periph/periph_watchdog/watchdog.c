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

/** @defgroup EXAMPLES_PERIPH_17XX40XX_WWDT LPC17xx/40xx Watchdog timer example
 * @ingroup EXAMPLES_PERIPH_17XX40XX
 * <b>Example description</b><br>
 * This example shows how to use Windowed watchdog timer to generate interrupts
 * to manage the watchdog timer. If the watchdog timer isn't 'fed' within it's
 * window time, it will reset the device. The LED will toggle states on each
 * watchdog feed event. You can also use a terminal program with the UART to
 * select the feed event. If the feed is disabled, the chip will reset.<br>
 *
 * The example works in one of 3 ways. If '1' is selected, WDT is fed by the
 * sysTick at 20Hz and should never time out (fast LED). If '2' is selcted, the
 * WDT is fed by the watchdog window interrupt (slow LED). If '3' is selected, the
 * watchdog times out and resets the device.<br>
 *
 * <b>Special connection requirements</b><br>
 * - Embedded Artists' LPC1788 Developer's Kit:<br>
 * - Embedded Artists' LPC4088 Developer's Kit:<br>
 * There are no special connection requirements for this example.<br>
 * - LPCXpresso LPC1769:<br>
 * Need to connect with base board for using RS232/UART port.<br>
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
#define FEED_ON_SYSTICKINT      0
#define FEED_ON_WARNINT         1
#define NO_FEED                 2

static volatile int wdtFeedState;
static volatile bool On = false;

/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/

/*****************************************************************************
 * Private functions
 ****************************************************************************/

/* WDT interrupt handler */
static void wdt_handle(void) {
	uint32_t wdtStatus = Chip_WWDT_GetStatus(LPC_WWDT);

	On = (bool) !On;

	/* The chip will reset before this happens, but if the WDT doesn't
	   have WWDT_MOD_WDRESET enabled, this will hit once */
	if (wdtStatus & WWDT_WDMOD_WDTOF) {
		/* A watchdog feed didn't occur prior to window timeout */
		Chip_WWDT_ClearStatusFlag(LPC_WWDT, WWDT_WDMOD_WDTOF);

		if (wdtFeedState == NO_FEED) {
			Chip_WWDT_Start(LPC_WWDT);	/* Needs restart */
		}
	}
#if !defined(CHIP_LPC175X_6X)
	/* Handle warning interrupt */
	if (wdtStatus & WWDT_WDMOD_WDINT) {
		/* A watchdog feed didn't occur prior to warning timeout */
		Chip_WWDT_ClearStatusFlag(LPC_WWDT, WWDT_WDMOD_WDINT);

		if (wdtFeedState == FEED_ON_WARNINT) {
			Chip_WWDT_Feed(LPC_WWDT);
		}
	}
#endif /*!defined(CHIP_LPC175X_6X)*/	
}

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
	wdt_handle();
}

/**
 * @brief	SysTick Interrupt Handler
 * @return	Nothing
 * @note	Systick interrupt handler feeds WWDT
 */
void SysTick_Handler(void)
{
	if (wdtFeedState == FEED_ON_SYSTICKINT) {
		On = (bool) !On;
		Chip_WWDT_Feed(LPC_WWDT);
	}
}

/**
 * @brief	Main entry point
 * @return	Nothing
 */
int main(void)
{
	uint8_t ch;
	uint32_t wdtFreq;

	Board_Init();

	/* Watchdog will be fed on each watchdog interrupt */
	wdtFeedState = FEED_ON_SYSTICKINT;

	/* Initialize WWDT and event router */
	Chip_WWDT_Init(LPC_WWDT);

#if defined(WATCHDOG_CLKSEL_SUPPORT)
	Chip_WWDT_SelClockSource(LPC_WWDT, WWDT_CLKSRC_WATCHDOG_PCLK);
	wdtFreq = Chip_Clock_GetPeripheralClockRate(SYSCTL_PCLK_WDT) / 4;
#else
	wdtFreq = WDT_OSC;
#endif
	
	/* Set watchdog feed time constant to 0.1s */
	Chip_WWDT_SetTimeOut(LPC_WWDT, wdtFreq / 10);
#if !defined(CHIP_LPC175X_6X)	
	/* Set watchdog warning time to 512 ticks after feed time constant
	   Set watchdog window time to 0.9s */
	Chip_WWDT_SetWarning(LPC_WWDT, 512);
	Chip_WWDT_SetWindow(LPC_WWDT, wdtFreq - (wdtFreq / 10));
#endif /*!defined(CHIP_LPC175X_6X)*/	

	/* Configure WWDT to reset on timeout */
	Chip_WWDT_SetOption(LPC_WWDT, WWDT_WDMOD_WDRESET);

	/* Clear watchdog warning and timeout interrupts */
	Chip_WWDT_ClearStatusFlag(LPC_WWDT, WWDT_WDMOD_WDTOF | WWDT_WDMOD_WDINT);

	/* Start watchdog */
	Chip_WWDT_Start(LPC_WWDT);

	/* Setup Systick for a 20Hz tick rate. Systick clock is clocked at
	   CPU core clock speed */
	SysTick_Config(Chip_Clock_GetSystemClockRate() / 20);

	/* Enable watchdog interrupt */
	NVIC_ClearPendingIRQ(WDT_IRQn);
	NVIC_EnableIRQ(WDT_IRQn);

	/* Watchdog test options */
	DEBUGOUT("Press '1' to enable watchdog feed on systick interrupt\n\r");
#if !defined(CHIP_LPC175X_6X)	
	DEBUGOUT("Press '2' to enable watchdog feed on warning interrupt\n\r");
#endif	
	DEBUGOUT("Press '3' to disable watchdog feed (will reset device)\n\r");

	while (1) {
		do {
			ch = DEBUGIN();
			Board_LED_Set(0, On);
		} while ((ch < '1') || (ch > '4'));

		switch (ch) {
		case '1':
		default:
			wdtFeedState = FEED_ON_SYSTICKINT;
			DEBUGOUT("Watchdog feed on systick interrupt\r\n");
			break;
#if !defined(CHIP_LPC175X_6X)
		case '2':
			wdtFeedState = FEED_ON_WARNINT;
			DEBUGOUT("Watchdog feed on warning interrupt\r\n");
			break;
#endif
		case '3':
			wdtFeedState = NO_FEED;
			DEBUGOUT("No feeding - board will reset\r\n");
			break;
		}
	}
}

/**
 * @}
 */
