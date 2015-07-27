/*
 * @brief Xpresso 1347 Sysinit file
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
 #include "string.h"

/** @ingroup BOARD_NXP_XPRESSO_1347
 * The System initialization code is called prior to the application and
 * initializes the board for run-time operation. Board initialization
 * includes clock setup and default pin muxing configuration.<br>
 *
 * With the exception of stack space, no RW memory is used for this call.<br>
 *
 * Clocking:<br>
 *  Main clocking set to System PLL 72MHz, System clock set at main clocking divided by 1<br>
 *  USB PLL is not setup.<br>
 * Pin muxing:<br>
 *   Sets up various pin mux functions for the board<br>
 * Memory:<br>
 *   There is no memory setup for this board.
 * @{
 */

/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/

/* IOCON pin definitions for pin muxing */
typedef struct {
	uint32_t port:8;		/* Pin port */
	uint32_t pin:8;			/* Pin number */
	uint32_t modefunc:16;	/* Pin mode */
} PINMUX_GRP_T;

/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/

/* Pin muxing table, only items that need changing from their default pin
   state are in this table. */
STATIC const PINMUX_GRP_T pinmuxing[] = {
	{0,  1,  (IOCON_FUNC1 | IOCON_MODE_INACT)},	/* PIO0_1 used for CLKOUT */
	{0,  2,  (IOCON_FUNC1 | IOCON_MODE_PULLUP)},	/* PIO0_2 used for SSEL */
	{0,  3,  (IOCON_FUNC1 | IOCON_MODE_INACT)},	/* PIO0_3 used for USB_VBUS */
	{0,  4,  (IOCON_FUNC1 | IOCON_SFI2C_EN)},	/* PIO0_4 used for SCL */
	{0,  5,  (IOCON_FUNC1 | IOCON_SFI2C_EN)},	/* PIO0_5 used for SDA */
	{0,  6,  (IOCON_FUNC1 | IOCON_MODE_INACT)},	/* PIO0_6 used for USB_CONNECT */
	{0,  8,  (IOCON_FUNC1 | IOCON_MODE_INACT)},	/* PIO0_8 used for MISO0 */
	{0,  9,  (IOCON_FUNC1 | IOCON_MODE_INACT)},	/* PIO0_9 used for MOSI0 */
	{0,  11, (IOCON_FUNC1 | IOCON_ADMODE_EN | IOCON_FILT_DIS)},/* PIO0_11 used for AD0 */
	{0,  18, (IOCON_FUNC1 | IOCON_MODE_INACT)},	/* PIO0_18 used for RXD */
	{0,  19, (IOCON_FUNC1 | IOCON_MODE_INACT)},	/* PIO0_19 used for TXD */
	{1,  29, (IOCON_FUNC1 | IOCON_MODE_INACT)},	/* PIO1_29 used for SCK0 */
};

/*****************************************************************************
 * Private functions
 ****************************************************************************/

/* Setup system clocking */
STATIC void SystemSetupClocking(void)
{
	int i;

	/* Powerup main oscillator */
	Chip_SYSCTL_PowerUp(SYSCTL_POWERDOWN_SYSOSC_PD);

	/* Wait an estimated 200us for OSC to be stablized, no status
	   indication, dummy wait */
	for (i = 0; i < 0x100; i++) {}

	/* Set system PLL input to main oscillator */
	Chip_Clock_SetSystemPllSource(SYSCTL_PLLCLKSRC_SYSOSC);

	/* Powerup system PLL */
	Chip_SYSCTL_PowerUp(SYSCTL_POWERDOWN_SYSPLL_PD);

	/* Setup PLL for main oscillator rate (FCLKIN = 12MHz) * 6 = 72MHz
	   MSEL = 5 (this is pre-decremented), PSEL = 1 (for P = 2)
	   FCLKOUT = FCLKIN * (MSEL + 1) = 12MHz * 6 = 72MHz
	   FCCO = FCLKOUT * 2 * P = 72MHz * 2 * 2 = 288MHz (within FCCO range) */
	Chip_Clock_SetupSystemPLL(5, 1);

	/* Wait for PLL to lock */
	while (!Chip_Clock_IsSystemPLLLocked()) {}

	/* Set system clock divider to 1 */
	Chip_Clock_SetSysClockDiv(1);

	/* Setup FLASH access to 3 clocks (72MHz clock) */
	Chip_FMC_SetFLASHAccess(FLASHTIM_72MHZ_CPU);

	/* Set main clock source to the system PLL. This will drive 72MHz
	   for the main clock and 72MHz for the system clock */
	Chip_Clock_SetMainClockSource(SYSCTL_MAINCLKSRC_PLLOUT);

	/* Enable IOCON clock */
	Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_IOCON);
}

/* Sets up system pin muxing */
STATIC void SystemSetupMuxing(void)
{
	int i;

	for (i = 0; i < (sizeof(pinmuxing) / sizeof(PINMUX_GRP_T)); i++) {
		Chip_IOCON_PinMuxSet(LPC_IOCON, pinmuxing[i].port, pinmuxing[i].pin,
						  pinmuxing[i].modefunc);
	}
}

/*****************************************************************************
 * Public functions
 ****************************************************************************/

/**
 * @brief	Setup the system
 * @return	none
 * @note	SystemInit() is called prior to the application and sets up system
 * clocking and any resources needed prior to the application
 * starting.
 */
void SystemInit(void)
{
	/* Booting from FLASH, so remap vector table to FLASH */
	Chip_SYSCTL_Map(REMAP_USER_FLASH_MODE);

	/* Setup system clocking and muxing */
	SystemSetupClocking();
	SystemSetupMuxing();
}
