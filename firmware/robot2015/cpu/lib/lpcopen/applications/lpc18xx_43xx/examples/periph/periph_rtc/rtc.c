/*
 * @brief RTC example
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

/** @defgroup EXAMPLES_PERIPH_18XX43XX_RTC LPC18xx/43xx RTC example
 * @ingroup EXAMPLES_PERIPH_18XX43XX
 * <b>Example description</b><br>
 * This example shows how to use the RTC, generate periodic interrupts,
 * and use the alarm interrupt.<br>
 *
 * The RTC is configured to generate an interrupt on each second field
 * register change. An interrupt is also configured when certain alarm field
 * values match the time field values. The LEDs will toggle states based on
 * the RTC events. You can also use a terminal program with the UART to
 * monitor RTC time and events.<br>
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

static volatile bool fIntervalReached;
static volatile bool fAlarmTimeMatched;
static volatile bool On0, On1;

/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/

/*****************************************************************************
 * Private functions
 ****************************************************************************/

/* Gets and shows the current time and date */
static void showTime(IP_RTC_TIME_T *pTime)
{
	DEBUGOUT("Time: %.2d:%.2d:%.2d %.2d/%.2d/%.4d\r\n", pTime->time[RTC_TIMETYPE_HOUR],
			 pTime->time[RTC_TIMETYPE_MINUTE],
			 pTime->time[RTC_TIMETYPE_SECOND],
			 pTime->time[RTC_TIMETYPE_MONTH],
			 pTime->time[RTC_TIMETYPE_DAYOFMONTH],
			 pTime->time[RTC_TIMETYPE_YEAR]);
}

/*****************************************************************************
 * Public functions
 ****************************************************************************/

/**
 * @brief	RTC interrupt handler
 * @return	Nothing
 */
void RTC_IRQHandler(void)
{
	uint32_t sec;

	/* Toggle heart beat LED for each second field change interrupt */
	if (Chip_RTC_GetIntPending(LPC_RTC, RTC_INT_COUNTER_INCREASE)) {
		/* Clear pending interrupt */
		Chip_RTC_ClearIntPending(LPC_RTC, RTC_INT_COUNTER_INCREASE);
		On0 = (bool) !On0;
		Board_LED_Set(0, On0);
	}

	/* display timestamp every 5 seconds in the background */
	sec = Chip_RTC_GetTime(LPC_RTC, RTC_TIMETYPE_SECOND);
	if (!(sec % 5)) {
		fIntervalReached = true;	/* set flag for background */
	}

	/* Check for alarm match */
	if (Chip_RTC_GetIntPending(LPC_RTC, RTC_INT_ALARM)) {
		/* Clear pending interrupt */
		Chip_RTC_ClearIntPending(LPC_RTC, RTC_INT_ALARM);
		fAlarmTimeMatched = true;	/* set alarm handler flag */
	}
}

/**
 * @brief	Main entry point
 * @return	Nothing
 */
int main(void)
{
	IP_RTC_TIME_T FullTime;

	Board_Init();

	fIntervalReached  = 0;
	fAlarmTimeMatched = 0;
	On0 = On1 = false;
	Board_LED_Set(2, false);

	DEBUGSTR("The RTC operates on a 1 Hz clock.\r\n" \
			 "Register writes can take up to 2 cycles.\r\n"	\
			 "It will take a few seconds to fully\r\n" \
			 "initialize it and start it running.\r\n\r\n");

	DEBUGSTR("We'll print a timestamp every 5 seconds.\r\n"	\
			 "...and another when the alarm occurs.\r\n");

	Chip_RTC_Init(LPC_RTC);

	/* Set current time for RTC 2:00:00PM, 2012-10-05 */
	FullTime.time[RTC_TIMETYPE_SECOND]  = 0;
	FullTime.time[RTC_TIMETYPE_MINUTE]  = 0;
	FullTime.time[RTC_TIMETYPE_HOUR]    = 14;
	FullTime.time[RTC_TIMETYPE_DAYOFMONTH]  = 5;
	FullTime.time[RTC_TIMETYPE_DAYOFWEEK]   = 5;
	FullTime.time[RTC_TIMETYPE_DAYOFYEAR]   = 279;
	FullTime.time[RTC_TIMETYPE_MONTH]   = 10;
	FullTime.time[RTC_TIMETYPE_YEAR]    = 2012;

	Chip_RTC_SetFullTime(LPC_RTC, &FullTime);

	/* Set ALARM time for 17 seconds from time */
	FullTime.time[RTC_TIMETYPE_SECOND]  = 17;
	Chip_RTC_SetFullAlarmTime(LPC_RTC, &FullTime);

	/* Set the RTC to generate an interrupt on each second */
	Chip_RTC_CntIncrIntConfig(LPC_RTC, RTC_AMR_CIIR_IMSEC, ENABLE);

	/* Enable matching for alarm for second, minute, hour fields only */
	Chip_RTC_AlarmIntConfig(LPC_RTC, RTC_AMR_CIIR_IMSEC | RTC_AMR_CIIR_IMMIN | RTC_AMR_CIIR_IMHOUR, ENABLE);

	/* Clear interrupt pending */
	Chip_RTC_ClearIntPending(LPC_RTC, RTC_INT_COUNTER_INCREASE | RTC_INT_ALARM);

	/* Enable RTC interrupt in NVIC */
	NVIC_EnableIRQ((IRQn_Type) RTC_IRQn);

	/* Enable RTC (starts increase the tick counter and second counter register) */
	Chip_RTC_Enable(LPC_RTC, ENABLE);

	/* Loop forever */
	while (1) {
		if (fIntervalReached) {	/* Every 5s */
			fIntervalReached = 0;

			On1 = (bool) !On1;
			Board_LED_Set(1, On1);

			/* read and display time */
			Chip_RTC_GetFullTime(LPC_RTC, &FullTime);
			showTime(&FullTime);
		}

		if (fAlarmTimeMatched) {
			fAlarmTimeMatched = false;

			/* announce event */
			DEBUGSTR("ALARM triggered!\r\n");
			Board_LED_Set(2, true);

			/* read and display time */
			Chip_RTC_GetFullTime(LPC_RTC, &FullTime);
			showTime(&FullTime);
		}
	}
}

/**
 * @}
 */
