/*
 * @brief LPC17xx/40xx EMC driver
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

#if !defined(CHIP_LPC175X_6X)

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

/* Dyanmic memory setup */
void Chip_EMC_Dynamic_Init(IP_EMC_DYN_CONFIG_T *Dynamic_Config)
{
	uint32_t ClkFreq;

	/* Note clocks must be enabled prior to this call */
	ClkFreq = Chip_Clock_GetEMCClockRate();

	IP_EMC_Dynamic_Init(LPC_EMC, Dynamic_Config, ClkFreq);
}

/* Static memory setup */
void Chip_EMC_Static_Init(IP_EMC_STATIC_CONFIG_T *Static_Config)
{
	uint32_t ClkFreq;

	/* Note clocks must be enabled prior to this call */
	ClkFreq = Chip_Clock_GetEMCClockRate();

	IP_EMC_Static_Init(LPC_EMC, Static_Config, ClkFreq);
}

#endif /* !defined(CHIP_LPC175X_6X) */
