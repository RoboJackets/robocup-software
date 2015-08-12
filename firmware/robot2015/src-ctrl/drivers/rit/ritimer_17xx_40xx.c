/*
 * @brief LPC17xx/40xx RITimer chip driver
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

#include "chip.h"

#if defined(CHIP_LPC175X_6X)

/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/

/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/

/*****************************************************************************
 * Private functions
 ****************************************************************************/

/*****************************************************************************
 * Public functions
 ****************************************************************************/

/* Initialize the RIT */
void Chip_RIT_Init(LPC_RITIMER_T *pRITimer)
{
	Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_RIT);
	IP_RIT_Init(pRITimer);
}

/* DeInitialize the RIT */
void Chip_RIT_DeInit(LPC_RITIMER_T *pRITimer)
{
	IP_RIT_DeInit(pRITimer);
	Chip_Clock_DisablePeriphClock(SYSCTL_CLOCK_RIT);
}

/* Set timer interval value */
void Chip_RIT_SetTimerInterval(LPC_RITIMER_T *pRITimer, uint32_t time_interval)
{
	uint32_t cmp_value;

	/* Determine aapproximate compare value based on clock rate and passed interval */
	cmp_value = (Chip_Clock_GetPeripheralClockRate(SYSCTL_PCLK_RIT) / 1000) * time_interval;

	/* Set timer compare value */
	Chip_RIT_SetCOMPVAL(pRITimer, cmp_value);

	/* Set timer enable clear bit to clear timer to 0 whenever
	   counter value equals the contents of RICOMPVAL */
	Chip_RIT_EnableCTRL(pRITimer, RIT_CTRL_ENCLR);
}

#endif /* defined(CHIP_LPC175X_6X) */
