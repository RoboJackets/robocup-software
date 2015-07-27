/*
 * @brief NXP LPC1769 Xpresso Sysinit file
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

/** @defgroup BOARD_NXP_XPRESSO_1769_SYSINIT NXP LPC1769 Xpresso board system init code
 * @ingroup BOARD_NXP_XPRESSO_1769
 * The System initialization code is called prior to the application and
 * initializes the board for run-time operation. Board initialization
 * includes clock setup and default pin muxing configuration.<br>
 *
 * With the exception of stack space, no RW memory is used for this call.<BR>
 *
 * Xpresso LPC1769 board setup<BR>
 *  Clocking:<BR>
 *   CPU setup for 120MHz of main oscillator and PLL0<BR>
 *   USB setup for 48MHz<BR>
 *  Pin muxing:<BR>
 *   Sets up various pin mux functions for the board (Ethernet, LEDs, etc.)<BR>
 *  Memory:<BR>
 *   There is no memory setup for this board.
 * @{
 */

/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/

/* SCR pin definitions for pin muxing */
typedef struct {
	uint8_t pingrp;		/* Pin group */
	uint8_t pinnum;		/* Pin number */
	uint8_t pincfg;		/* Pin configuration for SCU */
	uint8_t funcnum;	/* Function number */
} PINMUX_GRP_T;

/* Pin muxing configuration */
STATIC const PINMUX_GRP_T pinmuxing[] = {
	{0,  0,   IOCON_MODE_INACT,                   IOCON_FUNC2},	/* TXD3 */
	{0,  1,   IOCON_MODE_INACT,                   IOCON_FUNC2},	/* RXD3 */
	{0,  4,   IOCON_MODE_INACT,                   IOCON_FUNC2},	/* CAN-RD2 */
	{0,  5,   IOCON_MODE_INACT,                   IOCON_FUNC2},	/* CAN-TD2 */
	{0,  22,  IOCON_MODE_INACT,                   IOCON_FUNC0},	/* Led 0 */
	{0,  23,  IOCON_MODE_INACT,                   IOCON_FUNC1},	/* ADC 0 */
	{0,  26,  IOCON_MODE_INACT,                   IOCON_FUNC2},	/* DAC */
	/* ENET */
	{0x1, 0,  IOCON_MODE_INACT,                   IOCON_FUNC1},	/* ENET_TXD0 */
	{0x1, 1,  IOCON_MODE_INACT,                   IOCON_FUNC1},	/* ENET_TXD1 */
	{0x1, 4,  IOCON_MODE_INACT,                   IOCON_FUNC1},	/* ENET_TX_EN */
	{0x1, 8,  IOCON_MODE_INACT,                   IOCON_FUNC1},	/* ENET_CRS */
	{0x1, 9,  IOCON_MODE_INACT,                   IOCON_FUNC1},	/* ENET_RXD0 */
	{0x1, 10, IOCON_MODE_INACT,                   IOCON_FUNC1},	/* ENET_RXD1 */
	{0x1, 14, IOCON_MODE_INACT,                   IOCON_FUNC1},	/* ENET_RX_ER */
	{0x1, 15, IOCON_MODE_INACT,                   IOCON_FUNC1},	/* ENET_REF_CLK */
	{0x1, 16, IOCON_MODE_INACT,                   IOCON_FUNC1},	/* ENET_MDC */
	{0x1, 17, IOCON_MODE_INACT,                   IOCON_FUNC1},	/* ENET_MDIO */
};

/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/

/*****************************************************************************
 * Private functions
 ****************************************************************************/

/* Setup system clocking */
STATIC void SystemSetupClocking(void)
{
	/* CPU clock source starts with IRC */
	Chip_Clock_SetMainPllSource(SYSCTL_PLLCLKSRC_IRC);
	Chip_Clock_SetCPUClockSource(SYSCTL_CCLKSRC_SYSCLK);

	/* Enable main oscillator used for PLLs */
	LPC_SYSCTL->SCS = SYSCTL_OSCEC;
	while ((LPC_SYSCTL->SCS & SYSCTL_OSCSTAT) == 0) {}

	/* PLL0 clock source is 12MHz oscillator, PLL1 can only be the
	   main oscillator */
	Chip_Clock_SetMainPllSource(SYSCTL_PLLCLKSRC_MAINOSC);

	/* Setup PLL0 for a 480MHz clock. It is divided by CPU Clock Divider to create CPU Clock.
	   Input clock rate (FIN) is main oscillator = 12MHz
	   FCCO is selected for PLL Output and it must be between 275 MHz to 550 MHz.
	   FCCO = (2 * M * FIN) / N = integer multiplier of CPU Clock (120MHz) = 480MHz
	   N = 1, M = 480 * 1/(2*12) = 20 */
	Chip_Clock_SetupPLL(SYSCTL_MAIN_PLL, 19, 0);/* Multiply by 20, Divide by 1 */

	/* Enable PLL0 */
	Chip_Clock_EnablePLL(SYSCTL_MAIN_PLL, SYSCTL_PLL_ENABLE);

	/* Change the CPU Clock Divider setting for the operation with PLL0.
	   Divide value = (480/120) = 4 */
	Chip_Clock_SetCPUClockDiv(3);	/* pre-minus 1 */

	/* Change the USB Clock Divider setting for the operation with PLL0.
	   Divide value = (480/48) = 10 */
	Chip_Clock_SetUSBClockDiv(9);	/* pre-minus 1 */

	/* Wait for PLL0 to lock */
	while (!Chip_Clock_IsMainPLLLocked()) {}

	/* Connect PLL0 */
	Chip_Clock_EnablePLL(SYSCTL_MAIN_PLL, SYSCTL_PLL_ENABLE | SYSCTL_PLL_CONNECT);

	/* Wait for PLL0 to be connected */
	while (!Chip_Clock_IsMainPLLConnected()) {}

	/* Setup USB PLL1 for a 48MHz clock
	   Input clock rate (FIN) is main oscillator = 12MHz
	   PLL1 Output = USBCLK = 48MHz = FIN * MSEL, so MSEL = 4.
	   FCCO = USBCLK = USBCLK * 2 * P. It must be between 156 MHz to 320 MHz.
	   so P = 2 and FCCO = 48MHz * 2 * 2 = 192MHz */
	Chip_Clock_SetupPLL(SYSCTL_USB_PLL, 3, 1);	/* Multiply by 4, Divide by 2 */

#if 0	/* Use PLL1 output as USB Clock Source */
		/* Enable PLL1 */
	Chip_Clock_EnablePLL(SYSCTL_USB_PLL, SYSCTL_PLL_ENABLE);

	/* Wait for PLL1 to lock */
	while (!Chip_Clock_IsUSBPLLLocked()) {}

	/* Connect PLL1 */
	Chip_Clock_EnablePLL(SYSCTL_USB_PLL, SYSCTL_PLL_ENABLE | SYSCTL_PLL_CONNECT);

	/* Wait for PLL1 to be connected */
	while (!Chip_Clock_IsUSBPLLConnected()) {}
#endif

	/* Setup FLASH access to 5 clocks (120MHz clock) */
	Chip_FMC_SetFLASHAccess(FLASHTIM_120MHZ_CPU);
}

/* Sets up system pin muxing */
STATIC void SystemSetupMuxing(void)
{
	int i;

	/* Setup system level pin muxing */
	for (i = 0; i < (sizeof(pinmuxing) / sizeof(pinmuxing[0])); i++) {
		Chip_IOCON_PinMux(LPC_IOCON, pinmuxing[i].pingrp, pinmuxing[i].pinnum,
						  pinmuxing[i].pincfg, pinmuxing[i].funcnum);
	}
}

/*****************************************************************************
 * Public functions
 ****************************************************************************/

/**
 * @brief	Setup the system
 * @return	none
 * @note	SystemInit() is called prior to the application and sets up system
 *			clocking, memory, and any resources needed prior to the application
 *			starting.
 */
void SystemInit(void)
{
	unsigned int *pSCB_VTOR = (unsigned int *) 0xE000ED08;

#if defined(__IAR_SYSTEMS_ICC__)
	extern void *__vector_table;

	*pSCB_VTOR = (unsigned int) &__vector_table;
#elif defined(__CODE_RED)
	extern void *g_pfnVectors;

	*pSCB_VTOR = (unsigned int) &g_pfnVectors;
#elif defined(__ARMCC_VERSION)
	extern void *__Vectors;

	*pSCB_VTOR = (unsigned int) &__Vectors;
#endif

	/* Setup system clocking and memory. This is done early to allow the
	   application and tools to clear memory and use scatter loading to
	   external memory. */
	SystemSetupClocking();
	SystemSetupMuxing();
}

/**
 * @}
 */
