/*
 * @brief Multi-Rate Timer (MRT) example
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

/** @defgroup EXAMPLES_PERIPH_8XX_MRT LPC8xx Multi-Rate Timer example
 * @ingroup EXAMPLES_PERIPH_8XX
 * <b>Example description</b><br>
 * The MRT example demonstrates using the Multi-Rate Timer API functions.
 * This example configures the MRT using the GetIdleChannel function to
 * initialize a timer channel.  If a timer channel is not available because
 * all channels are in-use, the RED LED is illuminated.  Otherwise, a
 * timer channel is configured as a one-shot timer with a pseudo-random
 * value. Every pass through the MRT Interrupt Handler toggles a LED,
 * while another LED is toggled on every SysTick.<br>
 *
 * <b>Special connection requirements</b><br>
 * There are no special connection requirements for this example.<br>
 *
 * <b>Build procedures:</b><br>
 * @ref LPCOPEN_8XX_BUILDPROCS_IAR<br>
 * @ref LPCOPEN_8XX_BUILDPROCS_KEIL<br>
 * @ref LPCOPEN_8XX_BUILDPROCS_XPRESSO<br>
 *
 * <b>Supported boards and board setup:</b><br>
 * @ref LPCOPEN_8XX_BOARD_XPRESSO_812<br>
 *
 * <b>Submitting LPCOpen issues:</b><br>
 * @ref LPCOPEN_COMMUNITY
 * @{
 */

/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/

#define TICKRATE_HZ     (10)	/* 10 ticks per second */
#define RNG_MULTI       (69069)
#define NO_LO_BITS(x)   (x >> 8)

static uint32_t rng = 1234UL;

/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/

/*****************************************************************************
 * Private functions
 ****************************************************************************/

/* Random No. generator function */
static uint32_t RAND(void)
{
	rng *= RNG_MULTI;
	return (uint32_t) (NO_LO_BITS(rng));/* toss out the low order bits */
}

/*****************************************************************************
 * Public functions
 ****************************************************************************/

/**
 * @brief	Handle interrupt from SysTick timer
 * @return	Nothing
 */
void SysTick_Handler(void)
{
	Board_LED_Toggle(1);
	__SEV();
}

/**
 * @brief	Handle interrupt from MRT
 * @return	Nothing
 */
void MRT_IRQHandler(void)
{
	uint32_t idx;
    uint32_t int_pend;

    int_pend = Chip_MRT_GetIntPending();
	switch (int_pend) {
	case MRT0_INTFLAG:
		idx = 0;
		break;

	case MRT1_INTFLAG:
		idx = 1;
		break;

	case MRT2_INTFLAG:
		idx = 2;
		break;

	case MRT3_INTFLAG:
		idx = 3;
		break;

	default:/* should never get here */
		while (1) {
			Board_LED_Toggle(0);
		}
	}

	Chip_MRT_IntClear(&LPC_MRT->CHANNEL[idx]);
	Board_LED_Toggle(2);
	__SEV();
}

/**
 * @brief	MRT example main function
 * @return	Status (This function will not return)
 */
int main(void)
{
	/* a pointer to a timer */
	LPC_MRT_CHANNEL_T *timer;

	/* Generic Initialization */
	Board_Init();

	/* MRT Initialization */
	Chip_MRT_Init();

	/* Enable SysTick Timer */
	SysTick_Config(SystemCoreClock / TICKRATE_HZ);

    DEBUGSTR("LPC8xx MRT Example \r\n");

	/* Enable the interrupt for the MRT */
	NVIC_EnableIRQ(MRT_IRQn);

	while (1) {
		/* Check if IDLE channel available in MRT */
		if  (Chip_MRT_GetIdleChannel() != LPC_MRT_NO_IDLE_CHANNEL) {
			Board_LED_Set(0, false);

			/* Get channel structure of IDLE channel */
			timer = (LPC_MRT_CHANNEL_T *) (LPC_MRT_BASE + Chip_MRT_GetIdleChannel());

			/* Generate new pseudo-random number */
			RAND();

			/* Configure timer in single-shot mode */
			Chip_MRT_SetMode(timer, MRT_MODE_ONESHOT);

			/* Set some random amount of time interval */
			Chip_MRT_SetInterval(timer, rng / 100);

			/* Enable the channel */
			Chip_MRT_SetEnabled(timer);
		}
		else {
			/* indicate that no timer channel is available */
			Board_LED_Set(0, true);
		}
		__WFE();
	}

	return 0;
}

/**
 * @}
 */
