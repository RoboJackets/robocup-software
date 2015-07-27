/*
 * @brief LPC18xx/40xx EEPROM driver
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

#define EEPROM_CLOCK_DIV           1500000
#define EEPROM_READ_WAIT_STATE_VAL 0x58
#define EEPROM_WAIT_STATE_VAL      0x232

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
	uint32_t cclk;

	/* Setup EEPROM timing to 1500KHz based on CCLK rate */
	cclk = Chip_Clock_GetRate(CLK_MX_EEPROM);
	IP_EEPROM_Init(pEEPROM, cclk / EEPROM_CLOCK_DIV - 1);

	/* Setup EEPROM wait states*/
	IP_EEPROM_SetReadWaitState(pEEPROM, EEPROM_READ_WAIT_STATE_VAL);
	IP_EEPROM_SetWaitState(pEEPROM, EEPROM_WAIT_STATE_VAL);
}
