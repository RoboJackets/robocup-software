/*
 * @brief Event Monitor/Recorder example
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

/** @defgroup EXAMPLES_PERIPH_17XX40XX_RTC_EV LPC17xx/40xx Event Monitor/Recorder example
 * @ingroup EXAMPLES_PERIPH_17XX40XX
 * <b>Example description</b><br>
 * This example describes how to use Event Monitor/Recorder Block to
 * track event occurs on input pins.<br>
 *
 * After initialize RTC, set the current time for the RTC. When an event occurs
 * on an input channel, an interrupt is generated. The first and  last
 * timestamp of the event is printed out.<br>
 *
 * <b>Special connection requirements</b><br>
 * Connect desired event input channel pin to GND/VCC to generate events.<br>
 *  Event Input pins:<br>
 *               Channel         Pin         EA<br>
 *               0               P0.7        J5.17<br>
 *               1               P0.8        J3.19<br>
 *               2               P0.9        J5.18<br>
 *
 * <b>Build procedures:</b><br>
 * @ref LPCOPEN_17XX40XX_BUILDPROCS_KEIL<br>
 * @ref LPCOPEN_17XX40XX_BUILDPROCS_IAR<br>
 * @ref LPCOPEN_17XX40XX_BUILDPROCS_XPRESSO<br>
 *
 * <b>Supported boards and board setup:</b><br>
 * @ref LPCOPEN_17XX40XX_BOARD_EA1788<br>
 * @ref LPCOPEN_17XX40XX_BOARD_EA4088<br>
 *
 * <b>Submitting LPCOpen issues:</b><br>
 * @ref LPCOPEN_COMMUNITY
 * @{
 */

/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/
STATIC volatile bool fIntervalReached;

/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/

/*****************************************************************************
 * Private functions
 ****************************************************************************/

/* Shows the current time and date */
static void showTime(IP_RTC_TIME_T *pTime)
{
	DEBUGOUT("Time: %.2d:%.2d:%.2d %.2d/%.2d/%.4d\r\n",
			 pTime->time[RTC_TIMETYPE_HOUR],
			 pTime->time[RTC_TIMETYPE_MINUTE],
			 pTime->time[RTC_TIMETYPE_SECOND],
			 pTime->time[RTC_TIMETYPE_MONTH],
			 pTime->time[RTC_TIMETYPE_DAYOFMONTH],
			 pTime->time[RTC_TIMETYPE_YEAR]);
}

/* Shows timestamp value */
static void showTimeStamp(RTC_EV_TIMESTAMP_T *pTime)
{
	DEBUGOUT(" %.2d:%.2d:%.2d (DOY: %.3d)",
			 pTime->hour,
			 pTime->min,
			 pTime->sec,
			 pTime->dayofyear);
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
	RTC_EV_TIMESTAMP_T timestamp;
	uint8_t i = 0;
	uint32_t sec;

	if (Chip_RTC_GetIntPending(LPC_RTC, RTC_INT_COUNTER_INCREASE)) {
		/* Clear pending interrupt */
		Chip_RTC_ClearIntPending(LPC_RTC, RTC_INT_COUNTER_INCREASE);

		/* display timestamp every 5 seconds in the background */
		sec = Chip_RTC_GetTime(LPC_RTC, RTC_TIMETYPE_SECOND);
		if ((sec % 5) == 0) {
			fIntervalReached = true;	/* set flag for background */
		}
	}

	for (i = 0; i < RTC_EV_CHANNEL_NUM; i++) {
		if (Chip_RTC_EV_GetChannelStatus(LPC_RTC, (IP_RTC_EV_CHANNEL_T) i)) {
			uint8_t counter = 0;
			Chip_RTC_EV_GetFirstTimeStamp(LPC_RTC, (IP_RTC_EV_CHANNEL_T) i, &timestamp);
			DEBUGOUT("Event occurred at channel %.1d: ", i);
			showTimeStamp(&timestamp);
			Chip_RTC_EV_GetLastTimeStamp(LPC_RTC, (IP_RTC_EV_CHANNEL_T) i, &timestamp);
			DEBUGOUT(" - ");
			showTimeStamp(&timestamp);
			counter = Chip_RTC_EV_GetCounter(LPC_RTC, (IP_RTC_EV_CHANNEL_T) i);
			DEBUGOUT(", Counter = %.1d\r\n", counter);
			Chip_RTC_EV_ClearChannelStatus(LPC_RTC, (RTC_EV_CHANNEL_T) i);
		}
	}

	if (Chip_RTC_EV_GetStatus(LPC_RTC)) {
		Chip_RTC_EV_ClearStatus(LPC_RTC, Chip_RTC_EV_GetStatus(LPC_RTC));
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

	Board_RTC_EV_Init();

	fIntervalReached  = 0;

	DEBUGSTR("The RTC operates on a 1 Hz clock.\r\n" \
			 "Register writes can take up to 2 cycles.\r\n"	\
			 "It will take a few seconds to fully\r\n" \
			 "initialize it and start it running.\r\n\r\n");

	DEBUGSTR("We'll print a timestamp every 5 seconds.\r\n");
	DEBUGSTR("When a positive edge occurred on J5.18 or negative edge occurred on J5.17 pin, " \
			 "the first and last timestamp will be printed out.\r\n");

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

	/* Set the RTC to generate an interrupt on each second */
	Chip_RTC_CntIncrIntConfig(LPC_RTC, RTC_AMR_CIIR_IMSEC, ENABLE);

	/* Clear interrupt pending */
	Chip_RTC_ClearIntPending(LPC_RTC, RTC_INT_COUNTER_INCREASE);

	/* Enable RTC (starts increase the tick counter and second counter register) */
	Chip_RTC_Enable(LPC_RTC, ENABLE);

	/* Initialize Event Monitor/recorder */
	Chip_RTC_EV_Config(LPC_RTC, RTC_EV_CHANNEL_1, RTC_ERCTRL_POL_POSITIVE | RTC_ERCTRL_INPUT_EN);	/* Event on Negative edge */
	Chip_RTC_EV_Config(LPC_RTC, RTC_EV_CHANNEL_2, RTC_ERCTRL_POL_POSITIVE);	/* Event on Positive edge */
	Chip_RTC_EV_Config(LPC_RTC, RTC_EV_CHANNEL_3, RTC_ERCTRL_POL_NEGATIVE | RTC_ERCTRL_INPUT_EN);/* Event on Negative edge */

	/* Enable Event Monitor/Recorder */
	Chip_RTC_EV_SetMode(LPC_RTC, RTC_EV_MODE_ENABLE_64HZ);

	/* Enable RTC interrupt in NVIC */
	NVIC_EnableIRQ((IRQn_Type) RTC_IRQn);

	/* Loop forever */
	while (1) {
		if (fIntervalReached) {	/* Every 5s */
			fIntervalReached = false;

			/* read and display time */
			Chip_RTC_GetFullTime(LPC_RTC, &FullTime);
			showTime(&FullTime);
		}
	}
}

/**
 * @}
 */
