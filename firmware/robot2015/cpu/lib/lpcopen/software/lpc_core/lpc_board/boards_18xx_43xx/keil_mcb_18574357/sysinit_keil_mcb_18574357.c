/*
 * Copyright(C) NXP Semiconductors, 2012
 * All rights reserved.
 *
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
 * Permission to use, copy, modify, and distribute this software and its
 * documentation is hereby granted, under NXP Semiconductors' and its
 * licensor's relevant copyrights in the software, without fee, provided that it
 * is used in conjunction with NXP Semiconductors microcontrollers.  This
 * copyright, permission, and disclaimer notice must appear in all copies of
 * this code.
 */

#include "board.h"

/** @defgroup BOARD_KEIL_MCB_18574357_SYSINIT LPC1857 and LPC4357 Keil MCB board System Init code
 * @ingroup BOARD_KEIL_MCB_18574357
 * The System initialization code is called prior to the application and
 * initializes the board for run-time operation. Board initialization
 * for the Keil MCB boards includes clock setup, default pin muxing, and
 * memory configuration.
 *
 * With the exception of stack space, no RW memory is used for this call.
 *
 * LPC1857 and LPC4357 Keil MCB setup<BR>
 *  Clocking:<BR>
 *   All base clocks enabled by default (Save power by disabling un-needed clocks)<BR>
 *   CPU PLL set to maximum clock frequency (as defined by MAX_CLOCK_FREQ value)<BR>
 *   SPIFI FLASH clock setup for fastest speed<BR>
 *   CGU Dividers A, C, D and E are used for the LCD, USB, and SPIFI.
 *  Pin muxing:<BR>
 *   Sets up various pin mux functions for the board (Ethernet, LEDs, etc.)<BR>
 *   Sets up the external memory controller signals<BR>
 *  Memory:<BR>
 *   Sets up DRAM and NOR FLASH.<br>
 * <b><i>Note that this code is not for use with booting with a device on the
 * external EMC bus (ie, NOR FLASH). If booting from external FLASH on the EMC
 * bus is used, the chip's external address bus must be setup prior to
 * accessing any address beyond the first 16K.</i></b><br>
 * @{
 */

#ifndef CORE_M0
/* SCR pin definitions for pin muxing */
typedef struct {
	uint8_t pingrp;	/* Pin group */
	uint8_t pinnum;	/* Pin number */
	uint16_t modefunc;	/* Pin mode and function for SCU */
} PINMUX_GRP_T;

/* Structure for initial base clock states */
struct CLK_BASE_STATES {
	CHIP_CGU_BASE_CLK_T clk;	/* Base clock */
	CHIP_CGU_CLKIN_T clkin;	/* Base clock source, see UM for allowable souorces per base clock */
	bool autoblock_enab;/* Set to true to enable autoblocking on frequency change */
	bool powerdn;		/* Set to true if the base clock is initially powered down */
};

/* Initial base clock states are mostly on */
STATIC const struct CLK_BASE_STATES InitClkStates[] = {
	{CLK_BASE_SAFE, CLKIN_IRC, true, false},
	{CLK_BASE_APB1, CLKIN_MAINPLL, true, false},
	{CLK_BASE_APB3, CLKIN_MAINPLL, true, false},
	{CLK_BASE_USB0, CLKIN_USBPLL, true, true},
#if defined(CHIP_LPC43XX)
	{CLK_BASE_PERIPH, CLKIN_MAINPLL, true, false},
	{CLK_BASE_SPI, CLKIN_MAINPLL, true, false},
	{CLK_BASE_VADC, CLKIN_MAINPLL, true, true},
#endif
	{CLK_BASE_PHY_TX, CLKIN_ENET_TX, true, false},
#if defined(USE_RMII)
	{CLK_BASE_PHY_RX, CLKIN_ENET_TX, true, false},
#else
	{CLK_BASE_PHY_RX, CLKIN_ENET_RX, true, false},
#endif
	{CLK_BASE_SDIO, CLKIN_MAINPLL, true, false},
	{CLK_BASE_SSP0, CLKIN_MAINPLL, true, false},
	{CLK_BASE_SSP1, CLKIN_MAINPLL, true, false},
	{CLK_BASE_UART0, CLKIN_MAINPLL, true, false},
	{CLK_BASE_UART1, CLKIN_MAINPLL, true, false},
	{CLK_BASE_UART2, CLKIN_MAINPLL, true, false},
	{CLK_BASE_UART3, CLKIN_MAINPLL, true, false},
	{CLK_BASE_OUT, CLKINPUT_PD, true, false},
	{CLK_BASE_APLL, CLKINPUT_PD, true, false},
	{CLK_BASE_CGU_OUT0, CLKINPUT_PD, true, false},
	{CLK_BASE_CGU_OUT1, CLKINPUT_PD, true, false},

	/* Clocks derived from dividers */
	{CLK_BASE_LCD, CLKIN_IDIVC, true, false},
	{CLK_BASE_USB1, CLKIN_IDIVD, true, true}
};

/* SPIFI high speed pin mode setup */
STATIC const PINMUX_GRP_T spifipinmuxing[] = {
	{0x3, 3,  (SCU_PINIO_FAST | SCU_MODE_FUNC3)},
	{0x3, 4,  (SCU_PINIO_FAST | SCU_MODE_FUNC3)},
	{0x3, 5,  (SCU_PINIO_FAST | SCU_MODE_FUNC3)},
	{0x3, 6,  (SCU_PINIO_FAST | SCU_MODE_FUNC3)},
	{0x3, 7,  (SCU_PINIO_FAST | SCU_MODE_FUNC3)},
	{0x3, 8,  (SCU_PINIO_FAST | SCU_MODE_FUNC3)}
};

/* USB PLL pre-initialized setup values for 480MHz output rate */
static const CGU_USBAUDIO_PLL_SETUP_T usbPLLSetup = {
	0x0000601D,	/* Default control with main osc input, PLL disabled */
	0x06167FFA,	/* M-divider value for 480MHz output from 12MHz input */
	0x00000000,	/* N-divider value */
	0x00000000	/* Not applicable for USB PLL */
};

/* Audio PLL pre-initialized setup values for FIXME output rate */
// static const CGU_USBAUDIO_PLL_SETUP_T audioPLLSetup = {
//	0x0000601D, /* Default control with main osc input, PLL disabled */
//	0x06167FFA, /* M-divider value for FIXME output from 12MHz input */
//	0x00000000, /* N-divider value */
//	0x00000000  /* FIXME */
// };

/* Setup system clocking */
STATIC void SystemSetupClocking(void)
{
	int i;

	/* Setup FLASH acceleration to target clock rate prior to clock switch */
	Chip_CREG_SetFlashAcceleration(MAX_CLOCK_FREQ);

	/* Switch main system clocking to crystal */
	Chip_Clock_EnableCrystal();
	Chip_Clock_SetBaseClock(CLK_BASE_MX, CLKIN_CRYSTAL, true, false);

	/* Setup PLL for 100MHz and switch main system clocking */
	Chip_Clock_SetupMainPLLHz(CLKIN_CRYSTAL, CRYSTAL_MAIN_FREQ_IN, 100 * 1000000, 100 * 1000000);
	Chip_Clock_SetBaseClock(CLK_BASE_MX, CLKIN_MAINPLL, true, false);

	/* Setup PLL for maximum clock */
	Chip_Clock_SetupMainPLLHz(CLKIN_CRYSTAL, CRYSTAL_MAIN_FREQ_IN, MAX_CLOCK_FREQ, MAX_CLOCK_FREQ);

	/* Setup system base clocks and initial states. This won't enable and
	   disable individual clocks, but sets up the base clock sources for
	   each individual peripheral clock. */
	for (i = 0; i < (sizeof(InitClkStates) / sizeof(InitClkStates[0])); i++) {
		Chip_Clock_SetBaseClock(InitClkStates[i].clk, InitClkStates[i].clkin,
								InitClkStates[i].autoblock_enab, InitClkStates[i].powerdn);
	}

	/* Reset and enable 32Khz oscillator */
	LPC_CREG->CREG0 &= ~((1 << 3) | (1 << 2));
	LPC_CREG->CREG0 |= (1 << 1) | (1 << 0);

	/* SPIFI pin setup is done prior to setting up system clocking */
	for (i = 0; i < (sizeof(spifipinmuxing) / sizeof(spifipinmuxing[0])); i++) {
		Chip_SCU_PinMuxSet(spifipinmuxing[i].pingrp, spifipinmuxing[i].pinnum,
						   spifipinmuxing[i].modefunc);
	}

	/* Setup a divider E for main PLL clock switch SPIFI clock to that divider.
	   Divide rate is based on CPU speed and speed of SPI FLASH part. */
#if (MAX_CLOCK_FREQ > 180000000)
	Chip_Clock_SetDivider(CLK_IDIV_E, CLKIN_MAINPLL, 5);
#else
	Chip_Clock_SetDivider(CLK_IDIV_E, CLKIN_MAINPLL, 4);
#endif
	Chip_Clock_SetBaseClock(CLK_BASE_SPIFI, CLKIN_IDIVE, true, false);

	/* LCD with HX8347-D LCD Controller                                         */
	/* Attach main PLL clock to divider C with a divider of 2 */
	Chip_Clock_SetDivider(CLK_IDIV_C, CLKIN_MAINPLL, 2);

	/* Setup default USB PLL state for a 480MHz output and attach */
	Chip_Clock_SetupPLL(CLKIN_CRYSTAL, CGU_USB_PLL, &usbPLLSetup);

	/* USB1 needs a 60MHz clock. To get it, a divider of 4 and then 2 are
	   chained to make a divide by 8 function. Connect the output of
	   divider D to the USB1 base clock. */
	Chip_Clock_SetDivider(CLK_IDIV_A, CLKIN_USBPLL, 4);
	Chip_Clock_SetDivider(CLK_IDIV_D, CLKIN_IDIVA, 2);
	Chip_Clock_SetBaseClock(CLK_BASE_USB1, CLKIN_IDIVD, true, true);

	/* Setup default audio PLL state for a FIXME output */
	//	Chip_Clock_SetupPLL(CGU_AUDIO_PLL, &audioPLLSetup); // FIXME
}

STATIC const PINMUX_GRP_T pinmuxing[] = {
	/* RMII pin group */
	{0x1, 19,
	 (SCU_MODE_HIGHSPEEDSLEW_EN | SCU_MODE_MODE_PULLUP | SCU_MODE_INBUFF_EN | SCU_MODE_ZIF_DIS | SCU_MODE_FUNC0)},
	{0x0, 1,  (SCU_MODE_HIGHSPEEDSLEW_EN | SCU_MODE_MODE_PULLUP | SCU_MODE_ZIF_DIS | SCU_MODE_FUNC6)},
	{0x1, 18, (SCU_MODE_HIGHSPEEDSLEW_EN | SCU_MODE_MODE_PULLUP | SCU_MODE_ZIF_DIS | SCU_MODE_FUNC3)},
	{0x1, 20, (SCU_MODE_HIGHSPEEDSLEW_EN | SCU_MODE_MODE_PULLUP | SCU_MODE_ZIF_DIS | SCU_MODE_FUNC3)},
	{0x1, 17,
	 (SCU_MODE_HIGHSPEEDSLEW_EN | SCU_MODE_MODE_PULLUP | SCU_MODE_INBUFF_EN | SCU_MODE_ZIF_DIS | SCU_MODE_FUNC3)},
	{0xC, 1,  (SCU_MODE_HIGHSPEEDSLEW_EN | SCU_MODE_MODE_PULLUP | SCU_MODE_ZIF_DIS | SCU_MODE_FUNC3)},
	{0x1, 16,
	 (SCU_MODE_HIGHSPEEDSLEW_EN | SCU_MODE_MODE_PULLUP | SCU_MODE_INBUFF_EN | SCU_MODE_ZIF_DIS | SCU_MODE_FUNC7)},
	{0x1, 15,
	 (SCU_MODE_HIGHSPEEDSLEW_EN | SCU_MODE_MODE_PULLUP | SCU_MODE_INBUFF_EN | SCU_MODE_ZIF_DIS | SCU_MODE_FUNC3)},
	{0x0, 0,
	 (SCU_MODE_HIGHSPEEDSLEW_EN | SCU_MODE_MODE_PULLUP | SCU_MODE_INBUFF_EN | SCU_MODE_ZIF_DIS | SCU_MODE_FUNC2)},
	/* External data lines D0 .. D15 */
	{0x1, 7,
	 (SCU_MODE_MODE_PULLUP | SCU_MODE_INBUFF_EN | SCU_MODE_ZIF_DIS | SCU_MODE_HIGHSPEEDSLEW_EN | SCU_MODE_FUNC3)},
	{0x1, 8,
	 (SCU_MODE_MODE_PULLUP | SCU_MODE_INBUFF_EN | SCU_MODE_ZIF_DIS | SCU_MODE_HIGHSPEEDSLEW_EN | SCU_MODE_FUNC3)},
	{0x1, 9,
	 (SCU_MODE_MODE_PULLUP | SCU_MODE_INBUFF_EN | SCU_MODE_ZIF_DIS | SCU_MODE_HIGHSPEEDSLEW_EN | SCU_MODE_FUNC3)},
	{0x1, 10,
	 (SCU_MODE_MODE_PULLUP | SCU_MODE_INBUFF_EN | SCU_MODE_ZIF_DIS | SCU_MODE_HIGHSPEEDSLEW_EN | SCU_MODE_FUNC3)},
	{0x1, 11,
	 (SCU_MODE_MODE_PULLUP | SCU_MODE_INBUFF_EN | SCU_MODE_ZIF_DIS | SCU_MODE_HIGHSPEEDSLEW_EN | SCU_MODE_FUNC3)},
	{0x1, 12,
	 (SCU_MODE_MODE_PULLUP | SCU_MODE_INBUFF_EN | SCU_MODE_ZIF_DIS | SCU_MODE_HIGHSPEEDSLEW_EN | SCU_MODE_FUNC3)},
	{0x1, 13,
	 (SCU_MODE_MODE_PULLUP | SCU_MODE_INBUFF_EN | SCU_MODE_ZIF_DIS | SCU_MODE_HIGHSPEEDSLEW_EN | SCU_MODE_FUNC3)},
	{0x1, 14,
	 (SCU_MODE_MODE_PULLUP | SCU_MODE_INBUFF_EN | SCU_MODE_ZIF_DIS | SCU_MODE_HIGHSPEEDSLEW_EN | SCU_MODE_FUNC3)},
	{0x5, 4,
	 (SCU_MODE_MODE_PULLUP | SCU_MODE_INBUFF_EN | SCU_MODE_ZIF_DIS | SCU_MODE_HIGHSPEEDSLEW_EN | SCU_MODE_FUNC2)},
	{0x5, 5,
	 (SCU_MODE_MODE_PULLUP | SCU_MODE_INBUFF_EN | SCU_MODE_ZIF_DIS | SCU_MODE_HIGHSPEEDSLEW_EN | SCU_MODE_FUNC2)},
	{0x5, 6,
	 (SCU_MODE_MODE_PULLUP | SCU_MODE_INBUFF_EN | SCU_MODE_ZIF_DIS | SCU_MODE_HIGHSPEEDSLEW_EN | SCU_MODE_FUNC2)},
	{0x5, 7,
	 (SCU_MODE_MODE_PULLUP | SCU_MODE_INBUFF_EN | SCU_MODE_ZIF_DIS | SCU_MODE_HIGHSPEEDSLEW_EN | SCU_MODE_FUNC2)},
	{0x5, 0,
	 (SCU_MODE_MODE_PULLUP | SCU_MODE_INBUFF_EN | SCU_MODE_ZIF_DIS | SCU_MODE_HIGHSPEEDSLEW_EN | SCU_MODE_FUNC2)},
	{0x5, 1,
	 (SCU_MODE_MODE_PULLUP | SCU_MODE_INBUFF_EN | SCU_MODE_ZIF_DIS | SCU_MODE_HIGHSPEEDSLEW_EN | SCU_MODE_FUNC2)},
	{0x5, 2,
	 (SCU_MODE_MODE_PULLUP | SCU_MODE_INBUFF_EN | SCU_MODE_ZIF_DIS | SCU_MODE_HIGHSPEEDSLEW_EN | SCU_MODE_FUNC2)},
	{0x5, 3,
	 (SCU_MODE_MODE_PULLUP | SCU_MODE_INBUFF_EN | SCU_MODE_ZIF_DIS | SCU_MODE_HIGHSPEEDSLEW_EN | SCU_MODE_FUNC2)},
	{0xD, 2,
	 (SCU_MODE_MODE_PULLUP | SCU_MODE_INBUFF_EN | SCU_MODE_ZIF_DIS | SCU_MODE_HIGHSPEEDSLEW_EN | SCU_MODE_FUNC2)},
	{0xD, 3,
	 (SCU_MODE_MODE_PULLUP | SCU_MODE_INBUFF_EN | SCU_MODE_ZIF_DIS | SCU_MODE_HIGHSPEEDSLEW_EN | SCU_MODE_FUNC2)},
	{0xD, 4,
	 (SCU_MODE_MODE_PULLUP | SCU_MODE_INBUFF_EN | SCU_MODE_ZIF_DIS | SCU_MODE_HIGHSPEEDSLEW_EN | SCU_MODE_FUNC2)},
	{0xD, 5,
	 (SCU_MODE_MODE_PULLUP | SCU_MODE_INBUFF_EN | SCU_MODE_ZIF_DIS | SCU_MODE_HIGHSPEEDSLEW_EN | SCU_MODE_FUNC2)},
	{0xD, 6,
	 (SCU_MODE_MODE_PULLUP | SCU_MODE_INBUFF_EN | SCU_MODE_ZIF_DIS | SCU_MODE_HIGHSPEEDSLEW_EN | SCU_MODE_FUNC2)},
	{0xD, 7,
	 (SCU_MODE_MODE_PULLUP | SCU_MODE_INBUFF_EN | SCU_MODE_ZIF_DIS | SCU_MODE_HIGHSPEEDSLEW_EN | SCU_MODE_FUNC2)},
	{0xD, 8,
	 (SCU_MODE_MODE_PULLUP | SCU_MODE_INBUFF_EN | SCU_MODE_ZIF_DIS | SCU_MODE_HIGHSPEEDSLEW_EN | SCU_MODE_FUNC2)},
	{0xD, 9,
	 (SCU_MODE_MODE_PULLUP | SCU_MODE_INBUFF_EN | SCU_MODE_ZIF_DIS | SCU_MODE_HIGHSPEEDSLEW_EN | SCU_MODE_FUNC2)},
	{0xE, 5,
	 (SCU_MODE_MODE_PULLUP | SCU_MODE_INBUFF_EN | SCU_MODE_ZIF_DIS | SCU_MODE_HIGHSPEEDSLEW_EN | SCU_MODE_FUNC3)},
	{0xE, 6,
	 (SCU_MODE_MODE_PULLUP | SCU_MODE_INBUFF_EN | SCU_MODE_ZIF_DIS | SCU_MODE_HIGHSPEEDSLEW_EN | SCU_MODE_FUNC3)},
	{0xE, 7,
	 (SCU_MODE_MODE_PULLUP | SCU_MODE_INBUFF_EN | SCU_MODE_ZIF_DIS | SCU_MODE_HIGHSPEEDSLEW_EN | SCU_MODE_FUNC3)},
	{0xE, 8,
	 (SCU_MODE_MODE_PULLUP | SCU_MODE_INBUFF_EN | SCU_MODE_ZIF_DIS | SCU_MODE_HIGHSPEEDSLEW_EN | SCU_MODE_FUNC3)},
	{0xE, 9,
	 (SCU_MODE_MODE_PULLUP | SCU_MODE_INBUFF_EN | SCU_MODE_ZIF_DIS | SCU_MODE_HIGHSPEEDSLEW_EN | SCU_MODE_FUNC3)},
	{0xE, 10,
	 (SCU_MODE_MODE_PULLUP | SCU_MODE_INBUFF_EN | SCU_MODE_ZIF_DIS | SCU_MODE_HIGHSPEEDSLEW_EN | SCU_MODE_FUNC3)},
	{0xE, 11,
	 (SCU_MODE_MODE_PULLUP | SCU_MODE_INBUFF_EN | SCU_MODE_ZIF_DIS | SCU_MODE_HIGHSPEEDSLEW_EN | SCU_MODE_FUNC3)},
	{0xE, 12,
	 (SCU_MODE_MODE_PULLUP | SCU_MODE_INBUFF_EN | SCU_MODE_ZIF_DIS | SCU_MODE_HIGHSPEEDSLEW_EN | SCU_MODE_FUNC3)},
	/* Address lines A0 .. A23 */
	{0x2, 9,
	 (SCU_MODE_MODE_PULLUP | SCU_MODE_INBUFF_EN | SCU_MODE_ZIF_DIS | SCU_MODE_HIGHSPEEDSLEW_EN | SCU_MODE_FUNC3)},
	{0x2, 10,
	 (SCU_MODE_MODE_PULLUP | SCU_MODE_INBUFF_EN | SCU_MODE_ZIF_DIS | SCU_MODE_HIGHSPEEDSLEW_EN | SCU_MODE_FUNC3)},
	{0x2, 11,
	 (SCU_MODE_MODE_PULLUP | SCU_MODE_INBUFF_EN | SCU_MODE_ZIF_DIS | SCU_MODE_HIGHSPEEDSLEW_EN | SCU_MODE_FUNC3)},
	{0x2, 12,
	 (SCU_MODE_MODE_PULLUP | SCU_MODE_INBUFF_EN | SCU_MODE_ZIF_DIS | SCU_MODE_HIGHSPEEDSLEW_EN | SCU_MODE_FUNC3)},
	{0x2, 13,
	 (SCU_MODE_MODE_PULLUP | SCU_MODE_INBUFF_EN | SCU_MODE_ZIF_DIS | SCU_MODE_HIGHSPEEDSLEW_EN | SCU_MODE_FUNC3)},
	{0x1, 0,
	 (SCU_MODE_MODE_PULLUP | SCU_MODE_INBUFF_EN | SCU_MODE_ZIF_DIS | SCU_MODE_HIGHSPEEDSLEW_EN | SCU_MODE_FUNC2)},
	{0x1, 1,
	 (SCU_MODE_MODE_PULLUP | SCU_MODE_INBUFF_EN | SCU_MODE_ZIF_DIS | SCU_MODE_HIGHSPEEDSLEW_EN | SCU_MODE_FUNC2)},
	{0x1, 2,
	 (SCU_MODE_MODE_PULLUP | SCU_MODE_INBUFF_EN | SCU_MODE_ZIF_DIS | SCU_MODE_HIGHSPEEDSLEW_EN | SCU_MODE_FUNC2)},
	{0x2, 8,
	 (SCU_MODE_MODE_PULLUP | SCU_MODE_INBUFF_EN | SCU_MODE_ZIF_DIS | SCU_MODE_HIGHSPEEDSLEW_EN | SCU_MODE_FUNC3)},
	{0x2, 7,
	 (SCU_MODE_MODE_PULLUP | SCU_MODE_INBUFF_EN | SCU_MODE_ZIF_DIS | SCU_MODE_HIGHSPEEDSLEW_EN | SCU_MODE_FUNC3)},
	{0x2, 6,
	 (SCU_MODE_MODE_PULLUP | SCU_MODE_INBUFF_EN | SCU_MODE_ZIF_DIS | SCU_MODE_HIGHSPEEDSLEW_EN | SCU_MODE_FUNC2)},
	{0x2, 2,
	 (SCU_MODE_MODE_PULLUP | SCU_MODE_INBUFF_EN | SCU_MODE_ZIF_DIS | SCU_MODE_HIGHSPEEDSLEW_EN | SCU_MODE_FUNC2)},
	{0x2, 1,
	 (SCU_MODE_MODE_PULLUP | SCU_MODE_INBUFF_EN | SCU_MODE_ZIF_DIS | SCU_MODE_HIGHSPEEDSLEW_EN | SCU_MODE_FUNC2)},
	{0x2, 0,
	 (SCU_MODE_MODE_PULLUP | SCU_MODE_INBUFF_EN | SCU_MODE_ZIF_DIS | SCU_MODE_HIGHSPEEDSLEW_EN | SCU_MODE_FUNC2)},
	{0x6, 8,
	 (SCU_MODE_MODE_PULLUP | SCU_MODE_INBUFF_EN | SCU_MODE_ZIF_DIS | SCU_MODE_HIGHSPEEDSLEW_EN | SCU_MODE_FUNC1)},
	{0x6, 7,
	 (SCU_MODE_MODE_PULLUP | SCU_MODE_INBUFF_EN | SCU_MODE_ZIF_DIS | SCU_MODE_HIGHSPEEDSLEW_EN | SCU_MODE_FUNC1)},
	{0xD, 16,
	 (SCU_MODE_MODE_PULLUP | SCU_MODE_INBUFF_EN | SCU_MODE_ZIF_DIS | SCU_MODE_HIGHSPEEDSLEW_EN | SCU_MODE_FUNC2)},
	{0xD, 15,
	 (SCU_MODE_MODE_PULLUP | SCU_MODE_INBUFF_EN | SCU_MODE_ZIF_DIS | SCU_MODE_HIGHSPEEDSLEW_EN | SCU_MODE_FUNC2)},
	{0xE, 0,
	 (SCU_MODE_MODE_PULLUP | SCU_MODE_INBUFF_EN | SCU_MODE_ZIF_DIS | SCU_MODE_HIGHSPEEDSLEW_EN | SCU_MODE_FUNC3)},
	{0xE, 1,
	 (SCU_MODE_MODE_PULLUP | SCU_MODE_INBUFF_EN | SCU_MODE_ZIF_DIS | SCU_MODE_HIGHSPEEDSLEW_EN | SCU_MODE_FUNC3)},
	{0xE, 2,
	 (SCU_MODE_MODE_PULLUP | SCU_MODE_INBUFF_EN | SCU_MODE_ZIF_DIS | SCU_MODE_HIGHSPEEDSLEW_EN | SCU_MODE_FUNC3)},
	{0xE, 3,
	 (SCU_MODE_MODE_PULLUP | SCU_MODE_INBUFF_EN | SCU_MODE_ZIF_DIS | SCU_MODE_HIGHSPEEDSLEW_EN | SCU_MODE_FUNC3)},
	{0xE, 4,
	 (SCU_MODE_MODE_PULLUP | SCU_MODE_INBUFF_EN | SCU_MODE_ZIF_DIS | SCU_MODE_HIGHSPEEDSLEW_EN | SCU_MODE_FUNC3)},
	{0xA, 4,
	 (SCU_MODE_MODE_PULLUP | SCU_MODE_INBUFF_EN | SCU_MODE_ZIF_DIS | SCU_MODE_HIGHSPEEDSLEW_EN | SCU_MODE_FUNC3)},
	/* EMC control signals */
	{0x1, 4,
	 (SCU_MODE_MODE_PULLUP | SCU_MODE_INBUFF_EN | SCU_MODE_ZIF_DIS | SCU_MODE_HIGHSPEEDSLEW_EN | SCU_MODE_FUNC3)},
	{0x6, 6,
	 (SCU_MODE_MODE_PULLUP | SCU_MODE_INBUFF_EN | SCU_MODE_ZIF_DIS | SCU_MODE_HIGHSPEEDSLEW_EN | SCU_MODE_FUNC1)},
	{0xD, 13,
	 (SCU_MODE_MODE_PULLUP | SCU_MODE_INBUFF_EN | SCU_MODE_ZIF_DIS | SCU_MODE_HIGHSPEEDSLEW_EN | SCU_MODE_FUNC2)},
	{0xD, 10,
	 (SCU_MODE_MODE_PULLUP | SCU_MODE_INBUFF_EN | SCU_MODE_ZIF_DIS | SCU_MODE_HIGHSPEEDSLEW_EN | SCU_MODE_FUNC2)},
	{0x6, 9,
	 (SCU_MODE_MODE_PULLUP | SCU_MODE_INBUFF_EN | SCU_MODE_ZIF_DIS | SCU_MODE_HIGHSPEEDSLEW_EN | SCU_MODE_FUNC3)},
	{0x1, 6,
	 (SCU_MODE_MODE_PULLUP | SCU_MODE_INBUFF_EN | SCU_MODE_ZIF_DIS | SCU_MODE_HIGHSPEEDSLEW_EN | SCU_MODE_FUNC3)},
	{0x6, 4,
	 (SCU_MODE_MODE_PULLUP | SCU_MODE_INBUFF_EN | SCU_MODE_ZIF_DIS | SCU_MODE_HIGHSPEEDSLEW_EN | SCU_MODE_FUNC3)},
	{0x6, 5,
	 (SCU_MODE_MODE_PULLUP | SCU_MODE_INBUFF_EN | SCU_MODE_ZIF_DIS | SCU_MODE_HIGHSPEEDSLEW_EN | SCU_MODE_FUNC3)},
	{0x6, 11,
	 (SCU_MODE_MODE_PULLUP | SCU_MODE_INBUFF_EN | SCU_MODE_ZIF_DIS | SCU_MODE_HIGHSPEEDSLEW_EN | SCU_MODE_FUNC3)},
	{0x6, 12,
	 (SCU_MODE_MODE_PULLUP | SCU_MODE_INBUFF_EN | SCU_MODE_ZIF_DIS | SCU_MODE_HIGHSPEEDSLEW_EN | SCU_MODE_FUNC3)},
	{0x6, 10,
	 (SCU_MODE_MODE_PULLUP | SCU_MODE_INBUFF_EN | SCU_MODE_ZIF_DIS | SCU_MODE_HIGHSPEEDSLEW_EN | SCU_MODE_FUNC3)},
	{0xD, 0,
	 (SCU_MODE_MODE_PULLUP | SCU_MODE_INBUFF_EN | SCU_MODE_ZIF_DIS | SCU_MODE_HIGHSPEEDSLEW_EN | SCU_MODE_FUNC2)},
	{0xE, 13,
	 (SCU_MODE_MODE_PULLUP | SCU_MODE_INBUFF_EN | SCU_MODE_ZIF_DIS | SCU_MODE_HIGHSPEEDSLEW_EN | SCU_MODE_FUNC3)},
	{0x1, 3,  (SCU_PINIO_FAST | SCU_MODE_FUNC3)},
	{0x1, 4,  (SCU_PINIO_FAST | SCU_MODE_FUNC3)},
	{0x6, 6,  (SCU_PINIO_FAST | SCU_MODE_FUNC1)},
	{0x1, 5,  (SCU_PINIO_FAST | SCU_MODE_FUNC3)},
	{0x1, 6,  (SCU_PINIO_FAST | SCU_MODE_FUNC3)},
	/* Board LEDs */
	{0xD, 10, (SCU_MODE_MODE_PULLUP | SCU_MODE_FUNC4)},
	{0xD, 11, (SCU_MODE_MODE_PULLUP | SCU_MODE_FUNC4)},
	{0xD, 12, (SCU_MODE_MODE_PULLUP | SCU_MODE_FUNC4)},
	{0xD, 13, (SCU_MODE_MODE_PULLUP | SCU_MODE_FUNC4)},
	{0xD, 14, (SCU_MODE_MODE_PULLUP | SCU_MODE_FUNC4)},
	{0x9, 0,  (SCU_MODE_MODE_PULLUP | SCU_MODE_FUNC0)},
	{0x9, 1,  (SCU_MODE_MODE_PULLUP | SCU_MODE_FUNC0)},
	{0x9, 2,  (SCU_MODE_MODE_PULLUP | SCU_MODE_FUNC0)},
	/* SSP0 */
	{0xF, 0,  (SCU_PINIO_FAST | SCU_MODE_FUNC0)},
	{0xF, 1,  (SCU_PINIO_FAST | SCU_MODE_FUNC4)},
	{0xF, 2,  (SCU_PINIO_FAST | SCU_MODE_FUNC2)},
	{0xF, 3,  (SCU_PINIO_FAST | SCU_MODE_FUNC2)},
	/* LCD interface, 16bpp */
	{0x4, 1, ( SCU_MODE_MODE_INACT | SCU_MODE_FUNC5)},
	{0x4, 2, ( SCU_MODE_MODE_INACT | SCU_MODE_FUNC2)},
	{0x4, 5, ( SCU_MODE_MODE_INACT | SCU_MODE_FUNC2)},
	{0x4, 6, ( SCU_MODE_MODE_INACT | SCU_MODE_FUNC2)},
	{0x4, 7, ( SCU_MODE_MODE_INACT | SCU_MODE_FUNC0)},
	{0x4, 9, ( SCU_MODE_MODE_INACT | SCU_MODE_FUNC2)},
	{0x4, 10, (SCU_MODE_MODE_INACT | SCU_MODE_FUNC2)},
	{0x7, 0, ( SCU_MODE_MODE_INACT | SCU_MODE_FUNC0)},
	{0x7, 6, ( SCU_MODE_MODE_INACT | SCU_MODE_FUNC3)},
	{0x8, 3, ( SCU_MODE_MODE_INACT | SCU_MODE_FUNC3)},
	{0x8, 4, ( SCU_MODE_MODE_INACT | SCU_MODE_FUNC3)},
	{0x8, 5, ( SCU_MODE_MODE_INACT | SCU_MODE_FUNC3)},
	{0x8, 6, ( SCU_MODE_MODE_INACT | SCU_MODE_FUNC3)},
	{0x8, 7, ( SCU_MODE_MODE_INACT | SCU_MODE_FUNC3)},
	{0xB, 0, ( SCU_MODE_MODE_INACT | SCU_MODE_FUNC2)},
	{0xB, 1, ( SCU_MODE_MODE_INACT | SCU_MODE_FUNC2)},
	{0xB, 2, ( SCU_MODE_MODE_INACT | SCU_MODE_FUNC2)},
	{0xB, 3, ( SCU_MODE_MODE_INACT | SCU_MODE_FUNC2)},
	{0xB, 4, ( SCU_MODE_MODE_INACT | SCU_MODE_FUNC2)},
	{0xB, 5, ( SCU_MODE_MODE_INACT | SCU_MODE_FUNC2)},
	{0xB, 6, ( SCU_MODE_MODE_INACT | SCU_MODE_FUNC2)},
	/*  I2S  */
	{0x3, 0,  (SCU_PINIO_FAST | SCU_MODE_FUNC2)},
	{0x6, 0,  (SCU_PINIO_FAST | SCU_MODE_FUNC4)},
	{0x7, 2,  (SCU_PINIO_FAST | SCU_MODE_FUNC2)},
	{0x6, 2,  (SCU_PINIO_FAST | SCU_MODE_FUNC3)},
	{0x7, 1,  (SCU_PINIO_FAST | SCU_MODE_FUNC2)},
	{0x6, 1,  (SCU_PINIO_FAST | SCU_MODE_FUNC3)},
	/*  CCAN  */
	{0x3, 2,    (SCU_MODE_MODE_PULLUP | SCU_MODE_FUNC2)},	/* PE.3 CAN TD1 | SCU_MODE_FUNC1) */
	{0x3, 1,    (SCU_MODE_MODE_PULLUP | SCU_MODE_INBUFF_EN | SCU_MODE_FUNC2)},	/* PE.2	CAN RD1 | SCU_MODE_FUNC1) */
};

/* Pin clock mux values, re-used structure, value in first index is meaningless */
STATIC const PINMUX_GRP_T pinclockmuxing[] = {
	{0, 0,  (SCU_MODE_MODE_PULLUP | SCU_MODE_INBUFF_EN | SCU_MODE_ZIF_DIS | SCU_MODE_HIGHSPEEDSLEW_EN | SCU_MODE_FUNC0)},
	{0, 1,  (SCU_MODE_MODE_PULLUP | SCU_MODE_INBUFF_EN | SCU_MODE_ZIF_DIS | SCU_MODE_HIGHSPEEDSLEW_EN | SCU_MODE_FUNC0)},
	{0, 2,  (SCU_MODE_MODE_PULLUP | SCU_MODE_INBUFF_EN | SCU_MODE_ZIF_DIS | SCU_MODE_HIGHSPEEDSLEW_EN | SCU_MODE_FUNC0)},
	{0, 3,  (SCU_MODE_MODE_PULLUP | SCU_MODE_INBUFF_EN | SCU_MODE_ZIF_DIS | SCU_MODE_HIGHSPEEDSLEW_EN | SCU_MODE_FUNC0)},
};

/* Sets up system pin muxing */
STATIC void SystemSetupMuxing(void)
{
	int i;

	/* Setup system level pin muxing */
	for (i = 0; i < (sizeof(pinmuxing) / sizeof(pinmuxing[0])); i++) {
		Chip_SCU_PinMuxSet(pinmuxing[i].pingrp, pinmuxing[i].pinnum,
						   pinmuxing[i].modefunc);
	}
	/* Clock pins only, group field not used */
	for (i = 0; i < (sizeof(pinclockmuxing) / sizeof(pinclockmuxing[0])); i++) {
		Chip_SCU_ClockPinMuxSet(pinclockmuxing[i].pinnum, pinclockmuxing[i].modefunc);
	}
}

/* EMC clock delay */
#define CLK0_DELAY 7

/* Keil SDRAM timing and chip Config */
STATIC const IP_EMC_DYN_CONFIG_T MT48LC4M32_config = {
	EMC_NANOSECOND(64000000 / 4096),	/* Row refresh time */
	0x01,	/* Command Delayed */
	EMC_NANOSECOND(18),
	EMC_NANOSECOND(42),
	EMC_NANOSECOND(70),
	EMC_CLOCK(0x01),
	EMC_CLOCK(0x05),
	EMC_NANOSECOND(12),
	EMC_NANOSECOND(60),
	EMC_NANOSECOND(60),
	EMC_NANOSECOND(70),
	EMC_NANOSECOND(12),
	EMC_CLOCK(0x02),
	{
		{
			EMC_ADDRESS_DYCS0,	/* Keil Board uses DYCS0 for SDRAM */
			3,	/* RAS */

			EMC_DYN_MODE_WBMODE_PROGRAMMED |
			EMC_DYN_MODE_OPMODE_STANDARD |
			EMC_DYN_MODE_CAS_3 |
			EMC_DYN_MODE_BURST_TYPE_SEQUENTIAL |
			EMC_DYN_MODE_BURST_LEN_4,

			EMC_DYN_CONFIG_DATA_BUS_32 |
			EMC_DYN_CONFIG_LPSDRAM |
			EMC_DYN_CONFIG_4Mx32_4BANKS_12ROWS_8COLS |
			EMC_DYN_CONFIG_MD_SDRAM
		},
		{0, 0, 0, 0},
		{0, 0, 0, 0},
		{0, 0, 0, 0}
	}
};

/* Keil NorFlash timing and chip Config */
/* FIXME : Keil NOR FLASH not yet tested */
STATIC const IP_EMC_STATIC_CONFIG_T S29GL64N90_config = {
	0,
	EMC_STATIC_CONFIG_MEM_WIDTH_32 |
	EMC_STATIC_CONFIG_CS_POL_ACTIVE_LOW |
	EMC_STATIC_CONFIG_BLS_HIGH /* |
							      EMC_CONFIG_BUFFER_ENABLE*/,

	EMC_NANOSECOND(0),
	EMC_NANOSECOND(65),
	EMC_NANOSECOND(90),
	EMC_NANOSECOND(90),
	EMC_NANOSECOND(35),
	EMC_CLOCK(4)
};

/* Setup external memories */
STATIC void SystemSetupMemory(void)
{
	/* Setup EMC Delays */
	/* Move all clock delays together */
	LPC_SCU->EMCDELAYCLK = ((CLK0_DELAY) | (CLK0_DELAY << 4) | (CLK0_DELAY << 8) | (CLK0_DELAY << 12));

	/* Setup EMC Clock Divider for divide by 2 - this is done in both the CCU (clocking)
       and CREG. For frequencies over 120MHz, a divider of 2 must be used. For frequencies
	   less than 120MHz, a divider of 1 or 2 is ok. */
	Chip_Clock_EnableOpts(CLK_MX_EMC_DIV, true, true, 2);
	LPC_CREG->CREG6 |= (1 << 16);

	/* Enable EMC clock */
	Chip_Clock_Enable(CLK_MX_EMC);

	/* Init EMC Controller -Enable-LE mode */
	Chip_EMC_Init(1, 0);
	/* Init EMC Dynamic Controller */
	Chip_EMC_Dynamic_Init((IP_EMC_DYN_CONFIG_T *) &MT48LC4M32_config);
	/* Init EMC Static Controller CS0 */
	Chip_EMC_Static_Init((IP_EMC_STATIC_CONFIG_T *) &S29GL64N90_config);

	/* Enable Buffer for External Flash */
	LPC_EMC->STATICCONFIG0 |= 1 << 19;
}

#endif

/**
 * @brief	Setup the system
 *			SystemInit() is called prior to the application and sets up system
 *			clocking, memory, and any resources needed prior to the application
 *			starting.
 * @return	none
 */
void SystemInit(void)
{
#if defined(CORE_M3) || defined(CORE_M4)
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

#if defined(__FPU_PRESENT) && __FPU_PRESENT == 1
	fpuInit();
#endif

	/* Setup system clocking and memory. This is done early to allow the
	   application and tools to clear memory and use scatter loading to
	   external memory. */
	SystemSetupClocking();
	SystemSetupMuxing();
	SystemSetupMemory();
#endif
}

/**
 * @}
 */
