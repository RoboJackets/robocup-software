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

/** @defgroup BOARD_EA_DEVKIT_17884088_SYSINIT LPC1788 and LPC4088 Embedded Artists Development Kit System Init code
 * @ingroup BOARD_EA_DEVKIT_17884088
 * The System initialization code is called prior to the application and
 * initializes the board for run-time operation. Board initialization
 * for the EA1788/EA4088 boards includes clock setup, default pin muxing, and
 * memory configuration.
 *
 * With the exception of stack space, no RW memory is used for this call.
 *
 * LPC1788 and LPC4088 EA board setup<BR>
 *  Clocking:<BR>
 *   CPU setup for 120MHz off main oscillator and PLL0, EMC setup for 60MHz
 *   USB setup for 48MHz of main oscillator and PLL1
 *   Peripheral clock rate set to 60Mhz
 *   SPIFI clock disabled
 *  Pin muxing:<BR>
 *   Sets up various pin mux functions for the board (Ethernet, LEDs, etc.)<BR>
 *   Sets up the external memory controller signals<BR>
 *  Memory:<BR>
 *   Sets up DRAM.
 * @{
 */

/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/

/* SCR pin definitions for pin muxing */
typedef struct {
	uint8_t pingrp;		/* Pin group */
	uint8_t pinnum;		/* Pin number */
	uint32_t modefunc;	/* Pin mode and function */
} PINMUX_GRP_T;

/* Pin muxing configuration */
STATIC const PINMUX_GRP_T pinmuxing[] = {
	/* CAN RD1 and TD1 */
	{0x0, 0,  (IOCON_FUNC1 | IOCON_MODE_INACT)},
	{0x0, 1,  (IOCON_FUNC1 | IOCON_MODE_INACT)},
	/* UART 0 debug port (via USB bridge) */
	{0x0, 2,  (IOCON_FUNC1 | IOCON_MODE_INACT)},
	{0x0, 3,  (IOCON_FUNC1 | IOCON_MODE_INACT)},
	/* I2S */
	{0x0, 4,  (IOCON_FUNC1 | IOCON_MODE_INACT)},	/* I2S RX clock */
	{0x0, 5,  (IOCON_FUNC1 | IOCON_MODE_INACT)},	/* I2S RX WS */
	{0x0, 6,  (IOCON_FUNC1 | IOCON_MODE_INACT)},	/* I2S RX SDA */
	{0x0, 7,  (IOCON_FUNC1 | IOCON_MODE_INACT | IOCON_DIGMODE_EN)},	/* I2S TX clock */
	{0x0, 8,  (IOCON_FUNC1 | IOCON_MODE_INACT | IOCON_DIGMODE_EN)},	/* I2S TX WS */
	{0x0, 9,  (IOCON_FUNC1 | IOCON_MODE_INACT | IOCON_DIGMODE_EN)},	/* I2S TX SDA */
	/* SSP 0 */
	{0x0, 15, (IOCON_FUNC1 | IOCON_MODE_INACT)},	/* SSP CLK */
	{0x0, 16, (IOCON_FUNC1 | IOCON_MODE_INACT)},
	{0x0, 17, (IOCON_FUNC1 | IOCON_MODE_INACT)},
	{0x0, 18, (IOCON_FUNC1 | IOCON_MODE_INACT)},
	/* ADC */
	{0x0, 25, (IOCON_FUNC1 | IOCON_MODE_INACT | IOCON_ADMODE_EN)},
	/* DAC */
	{0x0, 26, (IOCON_FUNC2 | IOCON_DAC_EN | IOCON_HYS_EN | IOCON_MODE_PULLUP)},
	/* ENET */
	{0x1, 0, (IOCON_FUNC1 | IOCON_MODE_INACT)},
	{0x1, 1, (IOCON_FUNC1 | IOCON_MODE_INACT)},
	{0x1, 4, (IOCON_FUNC1 | IOCON_MODE_INACT)},
	{0x1, 8, (IOCON_FUNC1 | IOCON_MODE_INACT)},
	{0x1, 9, (IOCON_FUNC1 | IOCON_MODE_INACT)},
	{0x1, 10, (IOCON_FUNC1 | IOCON_MODE_INACT)},
#if defined(CHIP_LPC407X_8X)
	{0x1, 14, (IOCON_FUNC1 | IOCON_MODE_INACT | IOCON_DIGMODE_EN)},
#else
	{0x1, 14, (IOCON_FUNC1 | IOCON_MODE_INACT)},
#endif
	{0x1, 15, (IOCON_FUNC1 | IOCON_MODE_INACT)},
#if defined(CHIP_LPC407X_8X)
	{0x1, 16, (IOCON_FUNC1 | IOCON_MODE_INACT | IOCON_DIGMODE_EN)},
	{0x1, 17, (IOCON_FUNC1 | IOCON_MODE_INACT | IOCON_DIGMODE_EN)},
#else
	{0x1, 16, (IOCON_FUNC1 | IOCON_MODE_INACT)},
	{0x1, 17, (IOCON_FUNC1 | IOCON_MODE_INACT)},
#endif
	/* FIXME NOT COMPLETE */

	/* LEDs */
	{0x2, 26, (IOCON_FUNC0 | IOCON_MODE_INACT)},
	{0x2, 27, (IOCON_FUNC0 | IOCON_MODE_INACT)},
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

	/* Enable PBOOST for CPU clock over 100MHz */
	Chip_SYSCTL_EnableBoost();

	/* Enable main oscillator used for PLLs */
	LPC_SYSCTL->SCS = SYSCTL_OSCEC;
	while ((LPC_SYSCTL->SCS & SYSCTL_OSCSTAT) == 0) {}

	/* PLL0 clock source is 12MHz oscillator, PLL1 can only be the
	   main oscillator */
	Chip_Clock_SetMainPllSource(SYSCTL_PLLCLKSRC_MAINOSC);

	/* Setup PLL0 for a 120MHz clock
	   Input clock rate (FIN) is main oscillator = 12MHz
	   PLL output = 120MHz = FIN * MSEL = 120MHz, so MSEL = 10
	   FCCO must be between 156 MHz to 320 MHz, where FCCO = PLL output * 2 * P,
	   so P = 1 and FCCO = 120MHz * 2 * 1 = 240MHz */
	Chip_Clock_SetupPLL(SYSCTL_MAIN_PLL, 9, 0);	/* Multiply by 10, Divide by 1 */

	/* Enable PLL0 */
	Chip_Clock_EnablePLL(SYSCTL_MAIN_PLL, SYSCTL_PLL_ENABLE);

	/* Setup USB PLL1 for a 240MHz clock
	   Input clock rate (FIN) is main oscillator = 12MHz
	   PLL output = 240MHz = FIN * MSEL = 120MHz, so MSEL = 20
	   FCCO must be between 156 MHz to 320 MHz, where FCCO = PLL output * 2 * P,
	   so P = 1 and FCCO = 240MHz * 2 * 1 = 240MHz */
	Chip_Clock_SetupPLL(SYSCTL_USB_PLL, 19, 0);	/* Multiply by 20, Divide by 1 */

	/* Enable PLL0 */
	Chip_Clock_EnablePLL(SYSCTL_USB_PLL, SYSCTL_PLL_ENABLE);

	/* Since the USB PLL clock is 240MHz, we need to divide by 5 to get 48Mhz
	   as needed by the USB peripheral. Also set the USB divider source to
	     the USB PLL. USB clock rate = 240 / 5 = 48MHz */
	Chip_Clock_SetUSBClockSource(SYSCTL_USBCLKSRC_USBPLL);
	Chip_Clock_SetUSBClockDiv(5);

	/* PLL1 is disabled until needed */
	Chip_Clock_DisablePLL(SYSCTL_USB_PLL, SYSCTL_PLL_ENABLE);

	/* Wait for main (CPU) PLL0 to lock */
	while ((Chip_Clock_GetPLLStatus(SYSCTL_MAIN_PLL) & SYSCTL_PLLSTS_LOCKED) == 0) {}

	/* The CPU is still sourced from the SYSCLK, so set the CPU divider to
	   1 and switch it to the PLL0 clock */
	Chip_Clock_SetCPUClockDiv(1);
	Chip_Clock_SetCPUClockSource(SYSCTL_CCLKSRC_MAINPLL);

	/* Peripheral clocking will be derived from PLL0 with a divider of 2 (60MHz) */
	Chip_Clock_SetPCLKDiv(2);

#if defined(CHIP_LPC407X_8X)
	/* SPIFI clocking will be derived from Main PLL with a divider of 2 (60MHz) */
	Chip_Clock_SetSPIFIClockDiv(2);
	Chip_Clock_SetSPIFIClockSource(SYSCTL_SPIFICLKSRC_MAINPLL);
#endif
	
}

/* Sets up system pin muxing */
STATIC void SystemSetupMuxing(void)
{
	int i, j;

	Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_EMC);
	Chip_SYSCTL_PeriphReset(SYSCTL_RESET_IOCON);

	/* Setup data, address, and EMC control pins with high slew rate */
	for (i = 3; i <= 4; i++) {
		for (j = 0; j <= 31; j++) {
			Chip_IOCON_PinMuxSet(LPC_IOCON, (uint8_t) i, (uint8_t) j, (IOCON_FUNC1 | IOCON_FASTSLEW_EN));
		}
	}
	for (i = 16; i <= 31; i++) {
		Chip_IOCON_PinMuxSet(LPC_IOCON, 2, (uint8_t) i, (IOCON_FUNC1 | IOCON_FASTSLEW_EN));
	}

	/* Setup system level pin muxing */
	for (i = 0; i < (sizeof(pinmuxing) / sizeof(pinmuxing[0])); i++) {
		Chip_IOCON_PinMuxSet(LPC_IOCON, pinmuxing[i].pingrp, pinmuxing[i].pinnum,
							 pinmuxing[i].modefunc);
	}
}

/* EMC clock delay */
#define CLK0_DELAY 7

/* Keil SDRAM timing and chip Config */
STATIC const IP_EMC_DYN_CONFIG_T IS42S32800D_config = {
	EMC_NANOSECOND(64000000 / 4096),
	0x01,				/* Command Delayed */
	3,					/* tRP */
	7,					/* tRAS */
	EMC_NANOSECOND(70),	/* tSREX */
	EMC_CLOCK(0x01),	/* tAPR */
	EMC_CLOCK(0x05),	/* tDAL */
	EMC_NANOSECOND(12),	/* tWR */
	EMC_NANOSECOND(60),	/* tRC */
	EMC_NANOSECOND(60),	/* tRFC */
	EMC_NANOSECOND(70),	/* tXSR */
	EMC_NANOSECOND(12),	/* tRRD */
	EMC_CLOCK(0x02),	/* tMRD */
	{
		{
			EMC_ADDRESS_DYCS0,	/* EA Board uses DYCS0 for SDRAM */
			2,	/* RAS */

			EMC_DYN_MODE_WBMODE_PROGRAMMED |
			EMC_DYN_MODE_OPMODE_STANDARD |
			EMC_DYN_MODE_CAS_2 |
			EMC_DYN_MODE_BURST_TYPE_SEQUENTIAL |
			EMC_DYN_MODE_BURST_LEN_4,

			EMC_DYN_CONFIG_DATA_BUS_32 |
			EMC_DYN_CONFIG_LPSDRAM |
			EMC_DYN_CONFIG_8Mx16_4BANKS_12ROWS_9COLS |
			EMC_DYN_CONFIG_MD_SDRAM
		},
		{0, 0, 0, 0},
		{0, 0, 0, 0},
		{0, 0, 0, 0}
	}
};

/* NorFlash timing and chip Config */
STATIC const IP_EMC_STATIC_CONFIG_T SST39VF320_config = {
	0,
	EMC_STATIC_CONFIG_MEM_WIDTH_16 |
	EMC_STATIC_CONFIG_CS_POL_ACTIVE_LOW |
	EMC_STATIC_CONFIG_BLS_HIGH /* |
							      EMC_CONFIG_BUFFER_ENABLE*/,

	EMC_NANOSECOND(0),
	EMC_NANOSECOND(35),
	EMC_NANOSECOND(70),
	EMC_NANOSECOND(70),
	EMC_NANOSECOND(40),
	EMC_CLOCK(4)
};

/* NandFlash timing and chip Config */
STATIC const IP_EMC_STATIC_CONFIG_T K9F1G_config = {
	1,
	EMC_STATIC_CONFIG_MEM_WIDTH_8 |
	EMC_STATIC_CONFIG_CS_POL_ACTIVE_LOW |
	EMC_STATIC_CONFIG_BLS_HIGH /* |
							      EMC_CONFIG_BUFFER_ENABLE*/,

	EMC_NANOSECOND(0),
	EMC_NANOSECOND(35),
	EMC_NANOSECOND(70),
	EMC_NANOSECOND(70),
	EMC_NANOSECOND(40),
	EMC_CLOCK(4)
};

/* Setup external memories */
STATIC void SystemSetupMemory(void)
{
	/* Setup EMC Delays */
	/* Move all clock delays together */
	LPC_SYSCTL->EMCDLYCTL = (CLK0_DELAY) | (CLK0_DELAY << 8) | (CLK0_DELAY << 16 | (CLK0_DELAY << 24));

	/* Setup EMC Clock Divider for divide by 2 */
	/* Setup EMC clock for a divider of 2 from CPU clock. Enable EMC clock for
	   external memory setup of DRAM. */
	Chip_Clock_SetEMCClockDiv(SYSCTL_EMC_DIV2);
	Chip_SYSCTL_PeriphReset(SYSCTL_RESET_EMC);

	/* Init EMC Controller -Enable-LE mode- clock ratio 1:1 */
	Chip_EMC_Init(1, 0, 0);

	/* Init EMC Dynamic Controller */
	Chip_EMC_Dynamic_Init((IP_EMC_DYN_CONFIG_T *) &IS42S32800D_config);

	/* Init EMC Static Controller CS0 */
	Chip_EMC_Static_Init((IP_EMC_STATIC_CONFIG_T *) &SST39VF320_config);

	
	/* Init EMC Static Controller CS1 */
	Chip_EMC_Static_Init((IP_EMC_STATIC_CONFIG_T *) &K9F1G_config);

	/* EMC Shift Control */
	LPC_SYSCTL->SCS |= 1;
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
