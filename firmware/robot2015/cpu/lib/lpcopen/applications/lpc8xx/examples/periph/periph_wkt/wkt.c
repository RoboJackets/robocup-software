/*
 * @brief Self Wake-up Timer (WKT) example
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

/** @defgroup EXAMPLES_PERIPH_8XX_WKT LPC8xx Self Wakeup Timer (WKT) example
 * @ingroup EXAMPLES_PERIPH_8XX
 * <b>Example description</b><br>
 * The WKT example demonstrates using the Wake-up timer to wake-up the MCU from
 * low power states.<br>
 * When PIO0_6 pin is set to LOW level, the tests will be executed.
 * The application will demonstrate the following 4 sequential low power states:<br>
 * MCU_SLEEP,<br>
 * MCU_DEEP_SLEEP,<br>
 * MCU_POWER_DOWN,<br>
 * MCU_DEEP_POWER_DOWN<br>
 * The Red LED on the board will be toggling before the MCU enters
 * low power state & after it wakes up from the low power state.<br>
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

#define TICKRATE_HZ (10)/* 10 tick per second */

#define WAKEUP_COUNT_5s (5 * 10000)	/* 5 seconds (using 10Khz clock) */

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
 * @brief	Handle interrupt from Wake-up timer
 * @return	Nothing
 */
void WKT_IRQHandler(void)
{
	Chip_WKT_ClearIntStatus(LPC_WKT);
}

/**
 * @brief	main routine for wake-up timer example
 * @return	Function should not exit
 */
int main(void)
{
	bool port0_6;
	uint32_t i = 0, sleepCnt = 0x30000;
	CHIP_PMU_MCUPOWER_T  powerTest = PMU_MCU_SLEEP;

	/* Generic Initialization */
	Board_Init();

	/* Enable AHB clock to the GPIO domain to trigger test mode. */
	Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_GPIO);

	/* Alarm/wake timer as wakeup source */
	Chip_SYSCTL_EnablePeriphWakeup(SYSCTL_WAKEUP_WKTINT);

	/* Enable WKT clock */
	Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_WKT);

	/* Reset WKT */
	Chip_SYSCTL_PeriphReset(RESET_WKT);

	/* configure PIN0.6 as an input */
	Chip_GPIO_WriteDirBit(LPC_GPIO_PORT, LPC8XX_PORT_NUM, PIO6, false);

	/* configure PIN0.6 with pull-up */
	Chip_IOCON_PinSetMode(LPC_IOCON, PIO6, PIN_MODE_PULLUP);

	/* Enable WKT interrupt */
	NVIC_DisableIRQ(WKT_IRQn);
	NVIC_EnableIRQ(WKT_IRQn);

	while (1) {
		/* waiting for PIO0_6 going LOW to start Sleep test */
		do {
			port0_6 = Chip_GPIO_ReadPortBit(LPC_GPIO_PORT, LPC8XX_PORT_NUM, PIO6);
			i++;
			if (i > sleepCnt) {
				i = 0;
				Board_LED_Toggle(0);
			}
		} while (port0_6 == ENABLE);

		if (powerTest == PMU_MCU_SLEEP) {

			Chip_WKT_Start(LPC_WKT, WKT_CLKSRC_10KHZ, WAKEUP_COUNT_5s);
			Chip_PMU_Sleep(LPC_PMU, powerTest);

			Chip_WKT_Stop(LPC_WKT);
			powerTest = PMU_MCU_DEEP_SLEEP;
			sleepCnt = 0x50000;
		}
		else if (powerTest == PMU_MCU_DEEP_SLEEP) {

			Chip_WKT_Start(LPC_WKT, WKT_CLKSRC_10KHZ, WAKEUP_COUNT_5s);
			Chip_PMU_Sleep(LPC_PMU, powerTest);

			Chip_WKT_Stop(LPC_WKT);
			powerTest = PMU_MCU_POWER_DOWN;
			sleepCnt = 0x70000;
		}
		else if (powerTest == PMU_MCU_POWER_DOWN) {

			Chip_WKT_Start(LPC_WKT, WKT_CLKSRC_10KHZ, WAKEUP_COUNT_5s);
			Chip_PMU_Sleep(LPC_PMU, powerTest);

			Chip_WKT_Stop(LPC_WKT);
			powerTest = PMU_MCU_DEEP_PWRDOWN;
			sleepCnt = 0x90000;
		}
		else if (powerTest == PMU_MCU_DEEP_PWRDOWN) {

			Chip_WKT_Start(LPC_WKT, WKT_CLKSRC_10KHZ, WAKEUP_COUNT_5s);
			Chip_PMU_Sleep(LPC_PMU, powerTest);

			Chip_WKT_Stop(LPC_WKT);
			powerTest = PMU_MCU_SLEEP;
			sleepCnt = 0x30000;
		}
	}
	return 0;
}

/**
 * @}
 */
