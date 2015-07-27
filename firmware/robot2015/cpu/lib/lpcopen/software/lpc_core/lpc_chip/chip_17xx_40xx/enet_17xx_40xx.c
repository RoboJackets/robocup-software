/*
 * @brief LPC17xx/40xx ethernet driver
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

/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/

/* Divider index values for the MII PHY clock */
STATIC const uint8_t EnetClkDiv[] = {4, 6, 8, 10, 14, 20, 28, 36, 40, 44,
									 48, 52, 56, 60, 64};

/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/

/*****************************************************************************
 * Private functions
 ****************************************************************************/

/*****************************************************************************
 * Public functions
 ****************************************************************************/

/* Basic Ethernet interface initialization */
void Chip_ENET_Init(LPC_ENET_T *pENET)
{
	volatile uint32_t i;

	/* Enable Ethenet Clock */
	Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_ENET);
#if !defined(CHIP_LPC175X_6X)
	Chip_SYSCTL_PeriphReset(SYSCTL_RESET_ENET);
#endif

	/* Reset ethernet peripheral, small delay after reset */
	Chip_ENET_Reset(pENET);
	for (i = 0; i < 100; i++) {}

	IP_ENET_Init(pENET);
}

/* Find the divider index for a desired MII clock rate */
uint32_t Chip_ENET_FindMIIDiv(LPC_ENET_T *pENET, uint32_t clockRate)
{
	uint32_t tmp, divIdx = 0;

	/* Find desired divider value */
	tmp = Chip_Clock_GetENETClockRate() / clockRate;

	/* Determine divider index from desired divider */
	for (divIdx = 0; divIdx < (sizeof(EnetClkDiv) / sizeof(EnetClkDiv[0])); divIdx++) {
		/* Closest index, but not higher than desired rate */
		if (EnetClkDiv[divIdx] >= tmp) {
			return divIdx;
		}
	}

	/* Use maximum divider index */
	return (sizeof(EnetClkDiv) / sizeof(EnetClkDiv[0])) - 1;
}

/* Ethernet interface shutdown */
void Chip_ENET_DeInit(LPC_ENET_T *pENET)
{
	IP_ENET_DeInit(pENET);
	Chip_Clock_DisablePeriphClock(SYSCTL_CLOCK_ENET);
}
