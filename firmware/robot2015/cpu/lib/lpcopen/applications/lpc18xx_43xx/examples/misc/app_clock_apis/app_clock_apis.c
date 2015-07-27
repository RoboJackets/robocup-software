/*
 * @brief Clock APIs example
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

#include <stdlib.h>
#include <string.h>
#include "board.h"
#include "chip.h"

/** @defgroup EXAMPLES_MISC_18XX43XX_CLOCK_APIS LPC18xx/43xx Clock APIs example
 * @ingroup EXAMPLES_MISC_18XX43XX
 * <b>Example description</b><br>
 * This example demonstrates the use of Clock APIs to control the CGU settings.  This example uses UART
 * console to print the outputs.<br>
 *
 * <b>Special connection requirements</b><br>
 * There are no special connection requirements for this example.<br>
 *
 * <b>Build procedures:</b><br>
 * @ref LPCOPEN_18XX43XX_BUILDPROCS_KEIL<br>
 * @ref LPCOPEN_18XX43XX_BUILDPROCS_IAR<br>
 * @ref LPCOPEN_18XX43XX_BUILDPROCS_XPRESSO<br>
 *
 * <b>Supported boards and board setup:</b><br>
 * @ref LPCOPEN_18XX_BOARD_HITEX1850<br>
 * @ref LPCOPEN_43XX_BOARD_HITEX4350<br>
 * @ref LPCOPEN_18XX_BOARD_KEIL1857<br>
 * @ref LPCOPEN_43XX_BOARD_KEIL4357<br>
 *
 * <b>Submitting LPCOpen issues:</b><br>
 * @ref LPCOPEN_COMMUNITY
 * @{
 */

/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/

/* Input Clock structure */
typedef struct {
	CHIP_CGU_CLKIN_T clk_in;
	char clkin_name[16];
} CLKIN_NAME_T;

/* Base Clock Information structure */
typedef struct {
	CHIP_CGU_BASE_CLK_T clock;
	char clock_name[16];
} BASECLK_INFO_T;

/* Peripheral Clock Information structure */
typedef struct {
	CHIP_CCU_CLK_T per_clk;
	char clock_name[16];
} CCUCLK_INFO_T;

static char menu[] =
	"********************************************************************************\n\r"
	"Clock APIs demo \n\r"
	"Demonstrates how to setup, enable, disable clocks, reading the clock frequencies\n\r"
	"********************************************************************************\n\r";

static CLKIN_NAME_T clkin_info[] = {
	{CLKIN_32K, "Ext 32KHz", },		/*!< External 32KHz input */
	{CLKIN_IRC, "Int IRC", },		/*!< Internal IRC (12MHz) input */
	{CLKIN_ENET_RX, "ENET_RX", },	/*!< External ENET_RX pin input */
	{CLKIN_ENET_TX, "ENET_TX", },	/*!< External ENET_TX pin input */
	{CLKIN_CLKIN, "Ext GPCLKIN", },	/*!< External GPCLKIN pin input */
	{CLKIN_CRYSTAL, "Crystal", },	/*!< External (main) crystal pin input */
	{CLKIN_USBPLL, "USB PLL", },	/*!< Internal USB PLL input */
	{CLKIN_AUDIOPLL, "Audio PLL", },/*!< Internal Audio PLL input */
	{CLKIN_MAINPLL, "Main PLL", },	/*!< Internal Main PLL input */
	{CLKIN_IDIVA,  "IDIV A", },		/*!< Internal divider A input */
	{CLKIN_IDIVB,  "IDIV B", },		/*!< Internal divider B input */
	{CLKIN_IDIVC,  "IDIV C", },		/*!< Internal divider C input */
	{CLKIN_IDIVD,  "IDIV D", },		/*!< Internal divider D input */
	{CLKIN_IDIVE,  "IDIV E", },		/*!< Internal divider E input */
};

static BASECLK_INFO_T baseclk_info[] = {
	{CLK_BASE_SAFE, "SAFE", },
	{CLK_BASE_USB0, "USB0", },
#if defined(CHIP_LPC43XX)
	{CLK_BASE_PERIPH, "PERIPH", },
#endif
	{CLK_BASE_USB1, "USB1", },
	{CLK_BASE_MX, "MX_CORE", },
	{CLK_BASE_SPIFI, "SPIFI", },
#if defined(CHIP_LPC43XX)
	{CLK_BASE_SPI, "SPI", },
#endif
	{CLK_BASE_PHY_RX, "PHY_RX", },
	{CLK_BASE_PHY_TX, "PHY_TX", },
	{CLK_BASE_APB1, "APB1", },
	{CLK_BASE_APB3, "APB3", },
	{CLK_BASE_LCD, "LCD", },
#if defined(CHIP_LPC43XX)
	{CLK_BASE_VADC, "VADC", },
#endif
	{CLK_BASE_SDIO, "SDIO", },
	{CLK_BASE_SSP0, "SSP0", },
	{CLK_BASE_SSP1, "SSP1", },
	{CLK_BASE_UART0, "UART0", },
	{CLK_BASE_UART1, "UART1", },
	{CLK_BASE_UART2, "UART2", },
	{CLK_BASE_UART3, "UART3", },
	{CLK_BASE_OUT, "BASE OUT", },
	{CLK_BASE_CGU_OUT0, "CGU_OUT0", },
	{CLK_BASE_CGU_OUT1, "CGU_OUT1", },
};

static CCUCLK_INFO_T ccu_clk_info[] = {
	/* CCU1 clocks */
	{CLK_APB3_BUS, "APB3", },
	{CLK_APB3_I2C1, "I2C1", },
	{CLK_APB3_DAC, "DAC", },
	{CLK_APB3_ADC0, "ADC0", },
	{CLK_APB3_ADC1, "ADC1", },
	{CLK_APB3_CAN0, "CAN0", },
	{CLK_APB1_BUS,  "APB1 BUS", },
	{CLK_APB1_MOTOCON, "MOTORCON", },
	{CLK_APB1_I2C0, "I2C0", },
	{CLK_APB1_I2S, "I2S", },
	{CLK_APB1_CAN1, "CAN1", },
	{CLK_SPIFI, "SPIFI", },
	{CLK_MX_BUS, "MX BUS", },
	{CLK_MX_SPIFI, "MX SPIFI", },
	{CLK_MX_GPIO,  "GPIO", },
	{CLK_MX_LCD, "LCD", },
	{CLK_MX_ETHERNET, "ETHERNET", },
	{CLK_MX_USB0, "MX USB0", },
	{CLK_MX_EMC, "MX EMC", },
	{CLK_MX_SDIO, "MX SDIO", },
	{CLK_MX_DMA, "MX DMA", },
	{CLK_MX_MXCORE, "MX CORE", },
	{CLK_MX_SCT, "MX SCT", },
	{CLK_MX_USB1, "MX USB1", },
	{CLK_MX_EMC_DIV, "MX EMC DIV", },
	{CLK_MX_FLASHA, "MX FLASH A", },
	{CLK_MX_FLASHB, "MX FLASH B", },
#if defined(CHIP_LPC43XX)
	{CLK_M4_M0APP, "M4 M0 APP", },
	{CLK_MX_VADC, "VADC", },
#endif
	{CLK_MX_EEPROM, "EEPROM", },
	{CLK_MX_WWDT, "WWDT", },
	{CLK_MX_UART0, "MX UART0", },
	{CLK_MX_UART1, "MX UART1", },
	{CLK_MX_SSP0, "MX SSP0", },
	{CLK_MX_TIMER0, "TIMER0", },
	{CLK_MX_TIMER1, "TIMER1", },
	{CLK_MX_SCU, "SCU", },
	{CLK_MX_CREG, "CREG", },
	{CLK_MX_RITIMER, "RITIMER", },
	{CLK_MX_UART2, "MX UART2", },
	{CLK_MX_UART3, "MX UART3", },
	{CLK_MX_TIMER2, "TIMER2", },
	{CLK_MX_TIMER3, "TIMER3", },
	{CLK_MX_SSP1, "MX SSP1", },
	{CLK_MX_QEI, "QEI", },
#if defined(CHIP_LPC43XX)
	{CLK_PERIPH_BUS, "PERI BUS", },
	{CLK_PERIPH_CORE, "PERI CORE", },
	{CLK_PERIPH_SGPIO, "SGPIO", },
#endif
	/*{CLK_USB0, "BASE USB0", }, */
	/*{CLK_USB1, "BASE USB1", }, */
#if defined(CHIP_LPC43XX)
	{CLK_SPI, "SPI", },
	/*{CLK_VADC, "VADC", }, */
#endif

	/* CCU2 clocks */
	{CLK_APLL, "APLL", },
	{CLK_APB2_UART3, "UART3", },
	{CLK_APB2_UART2, "UART2", },
	{CLK_APB0_UART1, "UART1", },
	{CLK_APB0_UART0, "UART0", },
	{CLK_APB2_SSP1, "SSP1", },
	{CLK_APB0_SSP0, "SSP0", },
	{CLK_APB2_SDIO, "SDIO", },
};

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
 * @brief	Main entry point
 * @return	Nothing
 */
int main(void)
{
	bool bool_status;
	uint32_t mainpll_freq, clkin_frq, dive_value, baseclk_frq, perclk_frq;
	CHIP_CGU_CLKIN_T clk_in, base_input;
	bool autoblocken;
	bool powerdn;
	int i;
	volatile int k = 1;

	Board_Init();

	/* Print the Demo Information */
	DEBUGOUT(menu);

	/* Main PLL should be locked, Check if Main PLL is locked */
	DEBUGOUT("=========================================== \r\n");
	DEBUGOUT("PLL functions \r\n");
	DEBUGOUT("Main PLL : ");
	bool_status  = Chip_Clock_MainPLLLocked();
	if (bool_status == true) {
		DEBUGOUT("Locked\r\n");
	}
	else {
		DEBUGOUT("Not Locked\r\n");
		return 1;
	}

	/* Read Main PLL frequency in Hz */
	mainpll_freq  = Chip_Clock_GetMainPLLHz();
	if (mainpll_freq == 0) {
		DEBUGOUT("Error in reading Main PLL frequency \r\n");
		return 2;
	}
	DEBUGOUT("Main PLL Frequency in Hz : %d \r\n", mainpll_freq);
	DEBUGOUT("=========================================== \r\n");

	DEBUGOUT("=========================================== \r\n");
	DEBUGOUT("Clock Divider functions \r\n");
	/*
	 * Divider E divider is used for SPIFI, source is set to
	 * Main PLL in SysInit code.
	 * Read Divider E source & verify it
	 */
	clk_in = Chip_Clock_GetDividerSource(CLK_IDIV_E);
	if (clk_in != CLKIN_MAINPLL) {
		DEBUGOUT("Divider E source wrong %d \r\n", clk_in);
		return 3;
	}
	DEBUGOUT("Divider E source set to Main PLL \r\n");

	/*
	 * Divider E divider is used for SPIFI, divider value should be
	 * between 3 and 5 set in SysInit code.
	 * Read Divider E divider value & verify it
	 */
	dive_value = Chip_Clock_GetDividerDivisor(CLK_IDIV_E);
	if ( (dive_value < 3) && (dive_value > 5)) {
		DEBUGOUT("Divider E divider wrong %d \r\n", dive_value);
		return 4;
	}

	DEBUGOUT("Divider E divider value: %d \r\n", dive_value);
	DEBUGOUT("=========================================== \r\n");

	/*
	 * Read the frequencies of the input clock sources,
	 * print it on UART prompt
	 */
	DEBUGOUT("=========================================== \r\n");
	DEBUGOUT("Input clock frequencies \r\n");
	DEBUGOUT("=========================================== \r\n");
	for ( i = 0; i < (sizeof(clkin_info) / sizeof(CLKIN_NAME_T)); i++) {
		clkin_frq = Chip_Clock_GetClockInputHz(clkin_info[i].clk_in);
		DEBUGOUT(" %s Frequency : %d Hz \r\n", clkin_info[i].clkin_name, clkin_frq);
	}
	DEBUGOUT("=========================================== \r\n");

	/*
	 * Read the base clock settings & print on UART
	 */
	DEBUGOUT("=========================================== \r\n");
	DEBUGOUT("Base Clock Setting Information \r\n");
	DEBUGOUT("=========================================== \r\n");
	for ( i = 0; i < (sizeof(baseclk_info) / sizeof(BASECLK_INFO_T)); i++) {
		/* Read Base clock info, only if base clock is enabled */
		bool_status = Chip_Clock_IsBaseClockEnabled(baseclk_info[i].clock);
		if ( bool_status == true) {
			Chip_Clock_GetBaseClockOpts(baseclk_info[i].clock, &base_input,
										&autoblocken, &powerdn);
			/* Read Frequency of the base clock */
			baseclk_frq = Chip_Clock_GetBaseClocktHz(baseclk_info[i].clock);

			/* Print details on UART */
			DEBUGOUT("%s Input Clk: %d Base Clk Frq : %d Hz Auto block: %d Power down: %d \r\n",
					 baseclk_info[i].clock_name, base_input, baseclk_frq, autoblocken, powerdn);
		}
	}
	DEBUGOUT("=========================================== \r\n");

	/*
	 * Read the peripheral clock rate & print on UART
	 */
	DEBUGOUT("=========================================== \r\n");
	DEBUGOUT("Peripheral Clock Rates \r\n");
	DEBUGOUT("=========================================== \r\n");
	for ( i = 0; i < (sizeof(ccu_clk_info) / sizeof(CCUCLK_INFO_T)); i++) {
		/* Read Frequency of the peripheral clock */
		perclk_frq = Chip_Clock_GetRate(ccu_clk_info[i].per_clk);
		/* Print the per clock only if it is enabled */
		if (perclk_frq) {
			/* Print details on UART */
			DEBUGOUT("%s Per Frq : %d Hz \r\n", ccu_clk_info[i].clock_name,
					 perclk_frq);
		}
	}
	DEBUGOUT("=========================================== \r\n");

	while (k) ;

	return 0;
}

/**
 * @}
 */
