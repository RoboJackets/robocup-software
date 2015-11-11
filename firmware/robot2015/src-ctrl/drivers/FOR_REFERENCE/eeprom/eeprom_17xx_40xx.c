/*
 * @brief LPC17xx/40xx EEPROM driver
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

/* Initializes the EEPROM peripheral with specified parameter */
void Chip_EEPROM_Init(LPC_EEPROM_T *pEEPROM)
{
	uint32_t val, cclk;

	/* Setup EEPROM timing to 375KHz based on PCLK rate */
	cclk = Chip_Clock_GetSystemClockRate();
	IP_EEPROM_Init(pEEPROM, cclk / 375000 - 1);

	/* Setup EEPROM wait states to 15, 35, 35nS */
	val  = ((((cclk / 1000000) * 15) / 1000) + 1);
	val |= (((((cclk / 1000000) * 55) / 1000) + 1) << 8);
	val |= (((((cclk / 1000000) * 35) / 1000) + 1) << 16);
	IP_EEPROM_SetWaitState(pEEPROM, val);
}

#endif /* !defined(CHIP_LPC175X_6X) */
