/*
 * @brief LPC18xx/43xx SD/SDIO driver
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
#include "string.h"

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

/* Initializes the SD/MMC controller */
void Chip_SDIF_Init(LPC_SDMMC_T *pSDMMC)
{
	Chip_Clock_EnableOpts(CLK_MX_SDIO, true, true, 1);
	IP_SDMMC_Init(pSDMMC);
}

/* Shutdown the SD/MMC controller */
void Chip_SDIF_DeInit(LPC_SDMMC_T *pSDMMC)
{
	IP_SDMMC_DeInit(pSDMMC);
	Chip_Clock_Disable(CLK_MX_SDIO);
}

/* Returns the current SD status, clears pending ints, and disables all ints */
uint32_t Chip_SDIF_GetIntStatus(LPC_SDMMC_T *pSDMMC)
{
	uint32_t status;

	/* Get status and clear interrupts */
	status = IP_SDMMC_GetRawIntStatus(pSDMMC);
	IP_SDMMC_SetRawIntStatus(pSDMMC, status);
	IP_SDMMC_SetIntMask(pSDMMC, 0);

	return status;
}
