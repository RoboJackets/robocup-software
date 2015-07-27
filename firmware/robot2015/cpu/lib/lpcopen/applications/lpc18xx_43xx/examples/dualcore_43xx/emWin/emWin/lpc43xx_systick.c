/*
 * @brief	System Tick module functions for stand-alone configuration
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

 // FIXME - is this file needed? outside of norm.
/**
 * \file lpc43xx_systick.c
 * System Tick function source code
 */

#include "chip.h"
#include "board.h"

#include "GUI.h"

/** @defgroup EXAMPLE_DUALCORE_EMWIN LPC18xx/43xx emWin graphics dual core example using emWin library
 * @ingroup EXAMPLES_DUALCORE_43XX
 * <b>Example description</b><br>
 * The System tick module implements the system tick functionality for LPC43XX cores in
 * stand-alone configuration. The system tick functions are used in emWin graphics module.
 * For Cortex-M4 core, it will use the SysTick timer module.<br>
 * For Cortex-M0 core, it uses the RITimer module.<br>
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
/* Saved reference period */
static uint32_t saved_period;
static int16_t old_tmp_x = -1, old_tmp_y = -1;

/* Check every 20ms for TSC events */
#define TSC_CHECK_DELAY   (20)

#if (defined(CHIP_LPC43XX) && defined(CORE_M0))

#define RITIMER_IRQn_PRI  (255)

/* RITimer Reload value */
static uint32_t reload_val;
#endif

/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/

/**
 * @brief System Tick count value
 */
/* Saved total time in mS since timer was enabled */
volatile uint32_t systick_timems;

/**
 * Touch screen initialisation done flag (defined in emWin module)
 */
extern volatile int tsc_init_done;

/**
 * System Core rate (defined in LPC_BOARD library module)
 */
extern uint32_t SystemCoreClock;

/*****************************************************************************
 * Private functions
 ****************************************************************************/

/*****************************************************************************
 * Public functions
 ****************************************************************************/

#if (defined(CHIP_LPC43XX) && defined(CORE_M0))

/**
 * @brief	System Tick enable function
 * @param  period	:	System Tick period
 * @return None
 * This function will configure the system ticks as per mentioned period value.
 * For Cortex-M4 core, it will configure & enable the SysTick timer module.
 * For Cortex-M0 core it will configure & enable the RITimer module.
 */
void SysTick_Enable(uint32_t period)
{
	saved_period = period;

	/* Clear any pending interrupt */
	Chip_RIT_ClearInt(LPC_RITIMER);

	/* Calculate reload value */
	reload_val = ( SystemCoreClock / ( 1000 / period ) );
	Chip_RIT_SetCOMPVAL(LPC_RITIMER, Chip_RIT_GetCounter(LPC_RITIMER) + reload_val);/* Let it tick */

	/* Set the priority and enable the interrupt */
	NVIC_SetPriority((IRQn_Type) RITIMER_IRQn, RITIMER_IRQn_PRI);
	NVIC_EnableIRQ((IRQn_Type) RITIMER_IRQn);
}

/**
 * @brief	System Tick disable function
 * @return None
 * The function will disbale the system ticks
 * For Cortex-M4 core, it will disable in SysTick timer module.
 * For Cortex-M0 core it will disable in RITimer module.
 */
void SysTick_Disable(void)
{
	Chip_RIT_Disable(LPC_RITIMER);
}

/**
 * @brief	RITimer module IRQ handler function
 * @return None
 * The function will handle the RITimer interrupt events.
 * It will clear the RITimer interrupt, update the compare value for RITimer &
 * update the system tick count value.
 * If Touch screen is enabled, it will also check for the Touch screen events
 * at periodic intervals (interval specified by TSC_CHECK_DELAY)
 */
void RIT_IRQHandler(void)
{
	int16_t tmp_x = -1, tmp_y = -1;
	int16_t tmp_x1 = -1, tmp_y1 = -1;
	static uint8_t tsc_tick = 0;
	static uint8_t pressed = 0;
	bool touched;

	/* Clear RITimer Interrupt, Reload counter value */
	Chip_RIT_ClearInt(LPC_RITIMER);
	Chip_RIT_SetCOMPVAL(LPC_RITIMER, Chip_RIT_GetCounter(LPC_RITIMER) + reload_val);/* Reload value */

	/* Increment tick count */
	systick_timems += saved_period;

	/* If TSC enabled, store Touch event */
	if (tsc_init_done) {
		tsc_tick += saved_period;
		if (tsc_tick == TSC_CHECK_DELAY) {
			touched = Board_GetTouchPos((int16_t *) &tmp_x, (int16_t *) &tmp_y);
			if (touched == true) {
				if (pressed == 1) {
					if ((tmp_x >= 0) && (tmp_y > 0) && ((tmp_x != old_tmp_x) || (tmp_y != old_tmp_y))) {
						tmp_x1 = tmp_y;
						tmp_y1 = tmp_x;
						GUI_TOUCH_StoreState(320 - tmp_x1, tmp_y1);
						old_tmp_x = tmp_x;
						old_tmp_y = tmp_y;
					}
				}
				else {
					GUI_TOUCH_StoreState(320 - tmp_x1, tmp_y1);
					old_tmp_x = tmp_x;
					old_tmp_y = tmp_y;
					pressed = 1;
				}
			}
			else {
				if (pressed == 1) {
					GUI_TOUCH_StoreState(-1, -1);
					pressed = 0;
				}
			}
			tsc_tick = 0;
		}
	}
}

#else

/**
 * @brief	System Tick enable function
 * @param  period	:	System Tick period
 * @return None
 * This function will configure the system ticks as per mentioned period value.
 * For Cortex-M4 core, it will configure & enable the SysTick timer module.
 * For Cortex-M0 core it will configure & enable the the RITimer module.
 */
void SysTick_Enable(uint32_t period)
{
	saved_period = period;
	SysTick_Config((SystemCoreClock * period) / 1000);
}

/**
 * @brief	System Tick disable function
 * @return None
 * The function will disbale the system ticks
 */
void SysTick_Disable(void)
{
	SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;
}

/**
 * @brief	System Tick module IRQ handler function
 * @return None
 * The function will handle the System Tick module interrupt events.
 * It will update the system tick count value.
 * If Touch screen is enabled, it will also check for the Touch screen events
 * at periodic intervals (interval specified by TSC_CHECK_DELAY)
 */
void SysTick_Handler(void)
{
	int16_t tmp_x = -1, tmp_y = -1;
	int16_t tmp_x1 = -1, tmp_y1 = -1;
	static uint8_t tsc_tick = 0;
	static uint8_t pressed = 0;
	bool touched;

	/* Increment tick count */
	systick_timems += saved_period;

	/* If TSC enabled, store Touch event */
	if (tsc_init_done) {
		tsc_tick += saved_period;
		if (tsc_tick == TSC_CHECK_DELAY) {
			touched = Board_GetTouchPos((int16_t *) &tmp_x, (int16_t *) &tmp_y);
			if (touched == true) {
				if (pressed == 1) {
					if ((tmp_x >= 0) && (tmp_y > 0) && ((tmp_x != old_tmp_x) || (tmp_y != old_tmp_y))) {
						tmp_x1 = tmp_y;
						tmp_y1 = tmp_x;
						GUI_TOUCH_StoreState(320 - tmp_x1, tmp_y1);
						old_tmp_x = tmp_x;
						old_tmp_y = tmp_y;
					}
				}
				else {
					GUI_TOUCH_StoreState(320 - tmp_x1, tmp_y1);
					old_tmp_x = tmp_x;
					old_tmp_y = tmp_y;
					pressed = 1;
				}
			}
			else {
				if (pressed == 1) {
					GUI_TOUCH_StoreState(-1, -1);
					pressed = 0;
				}
			}
			tsc_tick = 0;
		}
	}
}

#endif

/**
 * @brief	Delay function
 * @param  ms	:	Delay period in milliseconds
 * @return None
 * Delay for the specified number of milliSeconds.
 * This function is used by LWIP stack in stand-alone configuration
 */
void msDelay(uint32_t ms)
{
	uint32_t to = ms + systick_timems;

	while (to > systick_timems) {}
}

/**
 * @brief	LWIP standalone mode time support
 * @return	Returns the current time in mS
 * Returns the current time in mS. This is needed for the LWIP timers
 */
uint32_t sys_now(void)
{
	return systick_timems;;
}

/**
 * @}
 */

/* --------------------------------- End Of File ------------------------------ */
