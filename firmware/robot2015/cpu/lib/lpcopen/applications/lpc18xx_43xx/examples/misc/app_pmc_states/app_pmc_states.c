/*
 * @brief Power Management Controller example
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

/** @defgroup EXAMPLES_MISC_18XX43XX_PMC_STATES LPC18xx/43xx Power Management Controller example
 * @ingroup EXAMPLES_MISC_18XX43XX
 * <b>Example description</b><br>
 * This example demonstrates the power states supported by PMC. The example demonstrates
 * steps to go to low power states & wake up from the states. <br>
 *
 * UART needs to be setup prior to running the example as the example takes input from
 * the UART console.<br>
 *
 * <b>Special connection requirements</b><br>
 *  - Hitex LPC1850EVA-A4-2 and LPC4350EVA-A4-2 boards<br>
 *    Connect a Serial cable (Straight cable) to the board's X1 (UART0) port and connect
 *    the other end to the host PC<br>
 *    Open TeraTerm, select the COM port corresponding to the Serial Port to which the board
 *    is connected, select 115200 as the baud rate, 1 stop bit, no parity and no flow control<br>
 *  - Keil MCB1857 and MCB4357 boards<br>
 *    Connect a Serial cable (Straight cable) to the board�s P11 (UART0/3) port and connect the
 *    other end to the host PC<br>
 *    Open TeraTerm, select the COM port corresponding to the Serial Port to which the board is
 *    connected, select 115200 as the baud rate, 1 stop bit, no parity and no flow control<br>
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
 * @ref LPCOPEN_18XX_BOARD_NGX1830<br>
 * @ref LPCOPEN_43XX_BOARD_NGX4330<br>
 *
 * <b>Submitting LPCOpen issues:</b><br>
 * @ref LPCOPEN_COMMUNITY
 * @{
 */

/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/

 #define RTC_ALARM_TIME 5	/*!< RTC Alarm in seconds */

/* Structure for initial base clock states */
struct CLK_BASE_STATES {
	CHIP_CGU_BASE_CLK_T clk;	/* Base clock */
	CHIP_CGU_CLKIN_T clkin;	/* Base clock source, see UM for allowable souorces per base clock */
	bool autoblock_enab;/* Set to true to enable autoblocking on frequency change */
	bool powerdn;		/* Set to true if the base clock is initially powered down */
};

static char menu[] =
	"\r\n==========================================================================\r\n"
	"NXP Semiconductors LPC18xx/43xx MCU Power Management Controller example      \r\n"
	"=============================================================================\r\n"
	"Select the power state test to be executed                           \r\n"
	" - Sleep state & Wake up test           : Press '1' to select        \r\n"
	" - Deep Sleep state & Wake up test      : Press '2' to select        \r\n"
	" - Power down state & Wake up test      : Press '3' to select        \r\n"
	" - Deep power down state & Wake up test : Press '4' to select        \r\n"
	" - Exit the demo                        : Press 'X' or 'x' to select \r\n"
	"=============================================================================\r\n";
static char menu1[] = "\r\nSelect the Wake up signal                      \r\n"
					  " - WAKEUP0 pin : Press 'W' or 'w' to select    \r\n"
					  " - RTC Alarm   : Press 'R' or 'r' to select    \r\n";

static char menu2[] = "PMC demo example terimnated \r\n";
static char menu3[] = "\r\nPress 'C' or 'c' to start demo...\r\n";

/* Initial base clock states are mostly on */
static struct CLK_BASE_STATES InitClkStates[] = {
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
	// {CLK_BASE_OUT, CLKINPUT_PD, true, false},
	// {CLK_BASE_APLL, CLKINPUT_PD, true, false},
	// {CLK_BASE_CGU_OUT0, CLKINPUT_PD, true, false},
	// {CLK_BASE_CGU_OUT1, CLKINPUT_PD, true, false},
	{CLK_BASE_LCD, CLKIN_IDIVC, true, false},
	{CLK_BASE_USB1, CLKIN_IDIVD, true, true},
};

/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/

/*****************************************************************************
 * Private functions
 ****************************************************************************/
static void PMC_Get_Wakeup_option(uint8_t *Wakeup_rtc)
{
	FlagStatus exitflag;
	uint8_t buffer = 0xFF;

	DEBUGOUT(menu1);

	exitflag = RESET;
	while (exitflag == RESET) {

		/* Get user input */
		buffer = DEBUGIN();

		if ((buffer == 'W') || (buffer == 'w')) {
			DEBUGOUT("WAKEUP0 pin selected \r\n");
			*Wakeup_rtc = 0;
			exitflag = SET;
		}

		if ((buffer == 'R') || (buffer == 'r')) {
			DEBUGOUT("RTC Alarm selected \r\n");
			*Wakeup_rtc = 1;
			exitflag = SET;
		}
	}
}

/**
 * Set IRC as source clock fo all the output clocks & power down
 * before going to 'Deep Sleep'/'Power Down'/'Deep Power Down' modes
 */
static void PMC_Pre_SleepPowerDown(void)
{
	int i;

#ifndef BOARD_NGX_XPLORER_18304330
	/* SDRAM in self refresh mode only for Keil & Hitex boards */
	LPC_EMC->DYNAMICCONTROL |= (1 << 2);
	while (!(LPC_EMC->STATUS & (1 << 2))) {}
#endif

	/* Shutdown perpheral clocks with wake up enabled */
	Chip_Clock_StartPowerDown();

	/* Get state of individual base clocks & store them for restoring.
	 * Sets up the IRC as base clock source
	 */
	for (i = 0; i < (sizeof(InitClkStates) / sizeof(InitClkStates[0])); i++) {
		/* Get the Base clock settings */
		Chip_Clock_GetBaseClockOpts(InitClkStates[i].clk, &InitClkStates[i].clkin,
									&InitClkStates[i].autoblock_enab, &InitClkStates[i].powerdn);

		/* Set IRC as clock input for all the base clocks */
		Chip_Clock_SetBaseClock(InitClkStates[i].clk, CLKIN_IRC, InitClkStates[i].autoblock_enab,
								InitClkStates[i].powerdn);
	}

	/* Set IRC as clock source for SPIFI */
	Chip_Clock_SetBaseClock(CLK_BASE_SPIFI, CLKIN_IRC, true, false);

	/* Set IRC as source clock for Core */
	Chip_Clock_SetBaseClock(CLK_BASE_MX, CLKIN_IRC, true, false);

#ifdef BOARD_KEIL_MCB_18574357
	/* Set Flash acceleration */
	Chip_CREG_SetFlashAcceleration(CRYSTAL_MAIN_FREQ_IN);
#endif

	/* Power down the main PLL */
	Chip_Clock_DisableMainPLL();
	Chip_Clock_DisablePLL(CGU_USB_PLL);
	Chip_Clock_DisableCrystal();
}

/**
 * Post wake up initialisation function
 */
static void PMC_Post_Wakeup(uint8_t buffer)
{
	int i;

#ifdef BOARD_KEIL_MCB_18574357
	/* Setup FLASH acceleration to target clock rate prior to clock switch */
	Chip_CREG_SetFlashAcceleration(MAX_CLOCK_FREQ);
#endif

	/* Reinitialise Clocks */
	Chip_Clock_SetBaseClock(CLK_BASE_MX, CLKIN_CRYSTAL, true, false);

	/* Setup PLL for 100MHz and switch main system clocking */
	Chip_Clock_SetupMainPLLHz(CLKIN_CRYSTAL, CRYSTAL_MAIN_FREQ_IN, 100 * 1000000, 100 * 1000000);
	Chip_Clock_SetBaseClock(CLK_BASE_MX, CLKIN_MAINPLL, true, false);

	/* Setup PLL for maximum clock */
	Chip_Clock_SetupMainPLLHz(CLKIN_CRYSTAL, CRYSTAL_MAIN_FREQ_IN, MAX_CLOCK_FREQ, MAX_CLOCK_FREQ);

	/* Restore the base clock states */
	for (i = 0; i < (sizeof(InitClkStates) / sizeof(InitClkStates[0])); i++) {
		Chip_Clock_SetBaseClock(InitClkStates[i].clk, InitClkStates[i].clkin,
								InitClkStates[i].autoblock_enab, InitClkStates[i].powerdn);
	}

	/* Reset and enable 32Khz oscillator */
	LPC_CREG->CREG0 &= ~((1 << 3) | (1 << 2));
	LPC_CREG->CREG0 |= (1 << 1) | (1 << 0);

	/* Setup a divider E for main PLL clock switch SPIFI clock to that divider.
	   Divide rate is based on CPU speed and speed of SPI FLASH part. */
#if (MAX_CLOCK_FREQ > 180000000)
	Chip_Clock_SetDivider(CLK_IDIV_E, CLKIN_MAINPLL, 5);
#else
	Chip_Clock_SetDivider(CLK_IDIV_E, CLKIN_MAINPLL, 4);
#endif
	Chip_Clock_SetBaseClock(CLK_BASE_SPIFI, CLKIN_IDIVE, true, false);

	/* Attach main PLL clock to divider C with a divider of 2 */
	Chip_Clock_SetDivider(CLK_IDIV_C, CLKIN_MAINPLL, 2);

	/* Re-Initialize debug UART
	 * � 115200bps
	 * � 8 data bit
	 * � No parity
	 * � 1 stop bit
	 * � No flow control
	 */
	DEBUGINIT();

#ifndef BOARD_NGX_XPLORER_18304330
	/* SDRAM in NORMAL mode, only for Keil & Hitex boards */
	LPC_EMC->DYNAMICCONTROL &= ~(1 << 2);
	while (LPC_EMC->STATUS & (1 << 2)) {}
#endif
}

/**
 * Event Router configure function
 */
static void PMC_Evrt_Configure(CHIP_EVRT_SRC_T Evrt_Src)
{
	/* Configure Interrupt signal from Evrt_Src pin to EVRT */
	Chip_EVRT_ConfigIntSrcActiveType(Evrt_Src, EVRT_SRC_ACTIVE_RISING_EDGE);

	/* Enable interrupt signal from Evrt_Src pin to EVRT */
	Chip_EVRT_SetUpIntSrc(Evrt_Src, ENABLE);

	/* Clear any pending interrupt */
	Chip_EVRT_ClrPendIntSrc(Evrt_Src);

	/* Disable EVRT interrupt in NVIC */
	NVIC_DisableIRQ(EVENTROUTER_IRQn);

	/* preemption = 1, sub-priority = 1 */
	NVIC_SetPriority(EVENTROUTER_IRQn, ((0x01 << 3) | 0x01));

	/* Enable Event Router interrupt in NVIC */
	NVIC_EnableIRQ(EVENTROUTER_IRQn);
}

/**
 * Power State handler function
 */
static void PMC_PwrState_Handler(uint8_t buffer, uint8_t Wake_RTC)
{
	CHIP_EVRT_SRC_T Evrt_Src;
	CHIP_PMC_PWR_STATE_T Pwr_state;
	uint8_t confirm = 0xFF;

	if (Wake_RTC) {
		/* Configure EVRT_SRC_RTC as wake up signal */
		Evrt_Src = EVRT_SRC_RTC;

		/* Disable interrupt signal from Evrt_Src pin to EVRT */
		Chip_EVRT_SetUpIntSrc(Evrt_Src, DISABLE);

		/* Initialize and configure RTC */
		Chip_RTC_Init(LPC_RTC);
		Chip_RTC_ResetClockTickCounter(LPC_RTC);
		Chip_RTC_SetTime(LPC_RTC, RTC_TIMETYPE_SECOND, 0);

		/* Set alarm time = RTC_ALARM_TIME seconds.
		 * So after each RTC_ALARM_TIME seconds, RTC will generate and wake-up system
		 */
		Chip_RTC_SetAlarmTime(LPC_RTC, RTC_TIMETYPE_SECOND, RTC_ALARM_TIME);
		Chip_RTC_CntIncrIntConfig(LPC_RTC, RTC_AMR_CIIR_IMSEC, DISABLE);

		/* Set the AMR for RTC_ALARM_TIME match alarm interrupt */
		Chip_RTC_AlarmIntConfig(LPC_RTC, RTC_AMR_CIIR_IMSEC, ENABLE);
		Chip_RTC_ClearIntPending(LPC_RTC, RTC_INT_ALARM);
	}
	else {
		Evrt_Src = EVRT_SRC_WAKEUP0;
	}

	/* Configure wake up signal */
	PMC_Evrt_Configure(Evrt_Src);

	/* Get confirmation from user to continue
	 * Print wake up signal information to user
	 */
	DEBUGOUT(menu3);
	while ( (confirm != 'C') && (confirm != 'c')) {
		confirm = DEBUGIN();
	}

	switch (buffer) {
	case '1':
		DEBUGOUT("Entering 'Sleep' state ...\r\n");
		if (Wake_RTC) {
			DEBUGOUT("Wait for %d seconds, RTC alarm will wake up from 'Sleep' mode \r\n", RTC_ALARM_TIME);
			Chip_RTC_Enable(LPC_RTC, ENABLE);
		}
		else {
			DEBUGOUT("Press WAKEUP0 button/Connect WAKEUP0 pin to 3.3V to exit 'Sleep' mode \r\n");
		}
		Chip_PMC_Sleep();
		Chip_RTC_DeInit(LPC_RTC);
		DEBUGOUT("Woken up \r\n");
		break;

	case '2':
		DEBUGOUT("Entering 'Deep Sleep' state ...\r\n");
		if (Wake_RTC) {
			DEBUGOUT("Wait for %d seconds, RTC alarm will wake up from 'Deep Sleep' mode \r\n", RTC_ALARM_TIME);
			Chip_RTC_Enable(LPC_RTC, ENABLE);
		}
		else {
			DEBUGOUT("Press WAKEUP0 button/Connect WAKEUP0 pin to 3.3V to exit 'Deep Sleep' mode \r\n");
		}
		Pwr_state = PMC_DeepSleep;

		/* Call Pre SleepPowerDown function */
		PMC_Pre_SleepPowerDown();

		/* Goto Deep Sleep mode */
		Chip_PMC_Set_PwrState(Pwr_state);

		/* Call Post Wake up Initialisation function */
		PMC_Post_Wakeup(buffer);
		DEBUGOUT("\r\nWoken up \r\n");
		break;

	case '3':
		DEBUGOUT("Entering 'Power Down' state ...\r\n");
		if (Wake_RTC) {
			DEBUGOUT("Wait for %d seconds, RTC alarm will wake up from 'Power Down' mode \r\n", RTC_ALARM_TIME);
			Chip_RTC_Enable(LPC_RTC, ENABLE);
		}
		else {
			DEBUGOUT("Press WAKEUP0 button/Connect WAKEUP0 pin to 3.3V to exit 'Power Down' mode \r\n");
		}
		Pwr_state = PMC_PowerDown;

		/* Call Pre SleepPowerDown function */
		PMC_Pre_SleepPowerDown();

		/* Goto Deep Sleep mode */
		Chip_PMC_Set_PwrState(Pwr_state);

		/* Call Post Wake up Initialisation function */
		PMC_Post_Wakeup(buffer);

		DEBUGOUT("\r\nWoken up \r\n");
		break;

	case '4':
		DEBUGOUT("Entering 'Deep Power Down' state ...\r\n");
		if (Wake_RTC) {
			DEBUGOUT("Wait for %d seconds, RTC alarm will wake up from 'Deep Power Down' mode \r\n", RTC_ALARM_TIME);
			Chip_RTC_Enable(LPC_RTC, ENABLE);
		}
		else {
			DEBUGOUT("Press WAKEUP0 button/Connect WAKEUP0 pin to 3.3V to exit 'Deep Power Down' mode \r\n");
		}
		Pwr_state = PMC_DeepPowerDown;

		/* Call Pre SleepPowerDown function */
		PMC_Pre_SleepPowerDown();

		/* Goto Deep Sleep mode */
		Chip_PMC_Set_PwrState(Pwr_state);

		/* Wake up from Deep power down state is as good as RESET */
		while (1) {}
		break;

	default:
		break;
	}
}

/*****************************************************************************
 * Public functions
 ****************************************************************************/

/**
 * @brief	Event router interrupt handler
 * @return	Nothing
 */
void EVRT_IRQHandler(void)
{
	if (Chip_EVRT_IsSourceInterrupting(EVRT_SRC_WAKEUP0)) {
		Chip_EVRT_ClrPendIntSrc(EVRT_SRC_WAKEUP0);
		NVIC_DisableIRQ(EVENTROUTER_IRQn);
	}

	if (Chip_EVRT_IsSourceInterrupting(EVRT_SRC_RTC)) {
		if (Chip_RTC_GetIntPending(LPC_RTC, RTC_INT_ALARM)) {
			Chip_RTC_ClearIntPending(LPC_RTC, RTC_INT_ALARM);
			Chip_EVRT_ClrPendIntSrc(EVRT_SRC_RTC);
			Chip_RTC_Enable(LPC_RTC, DISABLE);
			NVIC_DisableIRQ(EVENTROUTER_IRQn);
		}
	}
}

/**
 * @brief	Main entry point
 * @return	Nothing
 */
int main(void)
{
	FlagStatus exitflag;
	uint8_t buffer = 0xFF;
	uint8_t Wake_RTC = 0;

	Board_Init();

	/* Initialize the Event Router */
	Chip_EVRT_Init();

	/* Print user menu on UART console */
	DEBUGOUT(menu);

	exitflag = RESET;
	/* Read user option from UART prompt */
	while (exitflag == RESET) {

		/* Get user input */
		buffer = DEBUGIN();

		switch (buffer) {
		case '1':		/* Sleep/Wake up Mode */
			DEBUGOUT("'Sleep' state test selected \r\n");
			PMC_Get_Wakeup_option(&Wake_RTC);
			PMC_PwrState_Handler(buffer, Wake_RTC);
			DEBUGOUT(menu);
			break;

		case '2':		/* Deep sleep/Wakeup Mode */
			DEBUGOUT("'Deep Sleep' state test selected \r\n");
			PMC_Get_Wakeup_option(&Wake_RTC);
			PMC_PwrState_Handler(buffer, Wake_RTC);
			DEBUGOUT(menu);
			break;

		case '3':		/* Power Down/Wakeup Mode */
			DEBUGOUT("'Power Down' state test selected \r\n");
			PMC_Get_Wakeup_option(&Wake_RTC);
			PMC_PwrState_Handler(buffer, Wake_RTC);
			DEBUGOUT(menu);
			break;

		case '4':		/* Deep Power Down/Wakeup Mode */
			DEBUGOUT("'Deep Power Down' state test selected \r\n");
			PMC_Get_Wakeup_option(&Wake_RTC);
			PMC_PwrState_Handler(buffer, Wake_RTC);
			DEBUGOUT(menu);
			break;

		case 'X':
			DEBUGOUT(menu2);
			exitflag = SET;
			break;

		case 'x':
			DEBUGOUT(menu2);
			exitflag = SET;
			break;
		}
	}

	while (1) {}
}

/**
 * @}
 */
