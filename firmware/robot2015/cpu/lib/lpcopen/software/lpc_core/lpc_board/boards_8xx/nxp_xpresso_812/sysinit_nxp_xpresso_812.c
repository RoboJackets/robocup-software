/*
 * @brief NXP Xpresso LPC812 Sysinit file
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

/** @defgroup BOARD_NXP_XPRESSO_812_SYSINIT LPC812 NXP Xpresso board System Init code
 * @ingroup BOARD_NXP_XPRESSO_812
 * The System initialization code is called prior to the application and
 * initializes the board for run-time operation. Board initialization
 * for the NXP LPC812 board includes default pin muxing and clock setup
 * configuration.<br>
 *
 * With the exception of stack space, no RW memory is used for this call.<br>
 *
 * LPC812 setup<br>
 * Pin muxing:<br>
 *   Sets up various pin mux functions for the board. Enables CLKOUT on PIO0_1.<br>
 *  Clocking:<br>
 *   By default LPC8xx comes up using the IRC at a 1:1 ratio.<br>
 *   The system PLL is setup for 24MHz clocked from the main oscaillator and
 *   the system clock is switched to this rate.<br>
 *  Memory:<br>
 *   There is no memory setup for this board.<br>
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

/* Setup system clocking */
STATIC void SystemSetupClocking(void)
{
#if (LPC8XX_USE_XTAL_OSC == 1)
#if (CRYSTAL_MAIN_FREQ_IN >= 15000000)
	Chip_Clock_SetPLLBypass(false, true);	/* EXT oscillator >= 15MHz */
#else
	Chip_Clock_SetPLLBypass(false, false);	/* EXT oscillator < 15MHz */
#endif

	/* Turn on the SYSOSC by clearing the power down bit */
	Chip_SYSCTL_PowerUp(SYSCTL_SLPWAKE_SYSOSC_PD);

	/* Select the PLL input to the external oscillator */
	Chip_Clock_SetSystemPllSource(SYSCTL_PLLCLKSRC_SYSOSC);

#else
	/* Turn on the IRC by clearing the power down bit */
	Chip_SYSCTL_PowerUp(SYSCTL_SLPWAKE_IRC_PD);

	/* Select the PLL input in the IRC */
	Chip_Clock_SetSystemPllSource(SYSCTL_PLLCLKSRC_IRC);
#endif /* defined(LPC8XX_USE_XTAL_OSC) */

	/* Configure the PLL M and P dividers */
	/* Setup PLL for main oscillator rate (FCLKIN = 12MHz) * 2 = 324Hz
	   MSEL = 1 (this is pre-decremented), PSEL = 2 (for P = 4)
	   FCLKOUT = FCLKIN * (MSEL + 1) = 12MHz * 2 = 24MHz
	   FCCO = FCLKOUT * 2 * P = 24MHz * 2 * 4 = 192MHz (within FCCO range) */
	Chip_Clock_SetupSystemPLL(1, 2);

	/* Turn on the PLL by clearing the power down bit */
	Chip_SYSCTL_PowerUp(SYSCTL_SLPWAKE_SYSPLL_PD);

	/* Wait for PLL to lock */
	while (!Chip_Clock_IsSystemPLLLocked()) {}

	/* Set system clock divider to 1 */
	Chip_Clock_SetSysClockDiv(1);

	/* Setup FLASH access to 2 clocks (up to 30MHz) */
	Chip_FMC_SetFLASHAccess(FLASHTIM_30MHZ_CPU);

	/* Set main clock source to the system PLL. This will drive 24MHz
	   for the main clock and 24MHz for the system clock */
	Chip_Clock_SetMainClockSource(SYSCTL_MAINCLKSRC_PLLOUT);

	/* Select the CLKOUT clocking source */
	Chip_Clock_SetCLKOUTSource(SYSCTL_CLKOUTSRC_MAINSYSCLK, 1);
}

/* Sets up system pin muxing */
STATIC void SystemSetupMuxing(void)
{
#if (LPC8XX_USE_XTAL_OSC == 1)
	/* Use Switch Matrix Tool swm.c file for the Pin Enable 0 variable */
	LPC_SWM->PINENABLE0 = 0xffffff82UL;	/* XTALIN + XTALOUT + CLKOUT on PIO0_1 + ACMP_I1 */

	/* Configure the pins for XTALIN/XTALOUT. */
	LPC_IOCON->PIO0_8 &= ~(3 << 3);	/* Repeater Mode */
	LPC_IOCON->PIO0_9 &= ~(3 << 3);	/* Repeater Mode */

#else
	/* Use Switch Matrix Tool swm.c file for the Pin Enable 0 variable */
	LPC_SWM->PINENABLE0 = 0xffffffb2UL;		/* IRC + CLKOUT on PIO0_1 + ACMP_I1 */
#endif /* (LPC8XX_USE_XTAL_OSC == 1) */

	/* Assign the CLKOUT function to a pin */
	LPC_SWM->PINASSIGN[8] &= ~(0xFE << 16);	/* bits 23:16 = PIN0_x := 1 in this case */

	/* Configure the pin for CLKOUT on PIO0_1 */
	LPC_IOCON->PIO0_1 &= ~(3 << 3);	/* Repeater Mode */
}

/*****************************************************************************
 * Public functions
 ****************************************************************************/

/**
 * @brief	Setup the system
 * @return	none
 * @note	SystemInit() is called prior to the application and sets up system
 * clocking, memory, and any resources needed prior to the application
 * starting.
 */
void SystemInit(void)
{
	/* Booting from FLASH, so remap vector table to FLASH */
	Chip_SYSCTL_Map(REMAP_USER_FLASH_MODE);

	/* Enable the clock to the Switch Matrix */
	Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_SWM);

	/* Enable the clock for the IOCON */
	Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_IOCON);

	/* Setup system clocking and muxing */
	SystemSetupMuxing();
	SystemSetupClocking();

	/* Disable the clock to the Switch Matrix to save power */
	Chip_Clock_DisablePeriphClock(SYSCTL_CLOCK_SWM);

	/* IOCON clock left on, but may be turned off in no other IOCON
	   changes are needed */
}

/**
 * @}
 */
