/*
 * @brief NXP XPRESSO 11C24 Sysinit file
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

/** @ingroup BOARD_NXP_XPRESSO_11C24
 * The System initialization code is called prior to the application and
 * initializes the board for run-time operation. Board initialization
 * includes clock setup and default pin muxing configuration.<BR>
 * With the exception of stack space, no RW memory is used for this call.<BR>
 * Clocking:<BR>
 *  System and USB PLLs setup for 48MHz<BR>
 *  Main clocking set to System PLL 48MHz, System clock set at main clocking divided by 1<BR>
 * Pin muxing:<BR>
 *   Sets up various pin mux functions for the board<BR>
 * Memory:<BR>
 *   There is no memory setup for this board.<BR>
 * @{
 */

/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/

/* IOCON pin definitions for pin muxing */
typedef struct {
	uint32_t pin:8;			/* Pin number */
	uint32_t modefunc:24;	/* Function and mode */
} PINMUX_GRP_T;

/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/

/* Pin muxing table, only items that need changing from their default pin
   state are in this table. */
STATIC const PINMUX_GRP_T pinmuxing[] = {
	{(uint32_t) IOCON_PIO0_1,  (IOCON_FUNC1 | IOCON_MODE_INACT)},	/* PIO0_1 used for CLKOUT */
	{(uint32_t) IOCON_PIO0_2,  (IOCON_FUNC1 | IOCON_MODE_INACT)},	/* PIO0_2 used for SSEL */
	{(uint32_t) IOCON_PIO0_4,  (IOCON_FUNC1 | IOCON_SFI2C_EN)},	/* PIO0_4 used for SCL */
	{(uint32_t) IOCON_PIO0_5,  (IOCON_FUNC1 | IOCON_SFI2C_EN)},	/* PIO0_5 used for SDA */
	{(uint32_t) IOCON_PIO0_8,  (IOCON_FUNC1 | IOCON_MODE_INACT)},	/* PIO0_8 used for MISO */
	{(uint32_t) IOCON_PIO0_9,  (IOCON_FUNC1 | IOCON_MODE_INACT)},	/* PIO0_9 used for MOSI */
	{(uint32_t) IOCON_PIO0_11, (IOCON_FUNC1 | IOCON_ADMODE_EN)},	/* PIO0_11 used for AD0 */
	{(uint32_t) IOCON_PIO1_6,  (IOCON_FUNC1 | IOCON_MODE_INACT)},	/* PIO1_6 used for RXD */
	{(uint32_t) IOCON_PIO1_7,  (IOCON_FUNC1 | IOCON_MODE_INACT)},	/* PIO1_7 used for TXD */
	{(uint32_t) IOCON_PIO2_11, (IOCON_FUNC1 | IOCON_MODE_INACT)},	/* PIO0_6 used for SCK */
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

	/* Wait 200us for OSC to be stablized, no status
	   indication, dummy wait. */
	for (i = 0; i < 0x100; i++) {}

	/* Set system PLL input to main oscillator */
	Chip_Clock_SetSystemPllSource(SYSCTL_PLLCLKSRC_MAINOSC);

	/* Powerup system PLL */
	Chip_SYSCTL_PowerUp(SYSCTL_POWERDOWN_SYSPLL_PD);

	/* Setup PLL for main oscillator rate (FCLKIN = 12MHz) * 4 = 48MHz
	   MSEL = 3 (this is pre-decremented), PSEL = 1 (for P = 2)
	   FCLKOUT = FCLKIN * (MSEL + 1) = 12MHz * 4 = 48MHz
	   FCCO = FCLKOUT * 2 * P = 48MHz * 2 * 2 = 192MHz (within FCCO range) */
	Chip_Clock_SetupSystemPLL(3, 1);

	/* Wait for PLL to lock */
	while (!Chip_Clock_IsSystemPLLLocked()) {}

	/* Set system clock divider to 1 */
	Chip_Clock_SetSysClockDiv(1);

	/* Setup FLASH access to 3 clocks */
	Chip_FMC_SetFLASHAccess(FLASHTIM_50MHZ_CPU);

	/* Set main clock source to the system PLL. This will drive 48MHz
	   for the main clock and 48MHz for the system clock */
	Chip_Clock_SetMainClockSource(SYSCTL_MAINCLKSRC_PLLOUT);

	/* Enable IOCON clock */
	Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_IOCON);
}

/* Sets up system pin muxing */
STATIC void SystemSetupMuxing(void)
{
	int i;

	for (i = 0; i < (sizeof(pinmuxing) / sizeof(PINMUX_GRP_T)); i++) {
		Chip_IOCON_PinMuxSet(LPC_IOCON, (CHIP_IOCON_PIO_T) pinmuxing[i].pin,
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
