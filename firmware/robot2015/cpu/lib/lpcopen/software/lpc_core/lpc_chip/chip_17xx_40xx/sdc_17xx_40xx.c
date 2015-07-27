/*
 * @brief LPC17xx/40xx SD card driver
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

/* Initializes the SDC card controller */
void Chip_SDC_Init(LPC_SDC_T *pSDC)
{
	uint32_t i = 0;
	Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_SDC);
	Chip_SYSCTL_PeriphReset(SYSCTL_RESET_PCSDC);

	/* Initialize GPDMA controller */
	Chip_GPDMA_Init(LPC_GPDMA);

	/* Initialize SDC peripheral */
	IP_SDC_Init(pSDC);

	/* Power-up SDC Peripheral */
	IP_SDC_PowerControl(pSDC, SDC_POWER_UP, 0);

	/* delays for the supply output is stable*/
	for ( i = 0; i < 0x80000; i++ ) {}

	Chip_SDC_SetClock(pSDC, SDC_IDENT_CLOCK_RATE);
	IP_SDC_ClockControl(pSDC, SDC_CLOCK_ENABLE, ENABLE);

	/* Power-on SDC Interface */
	IP_SDC_PowerControl(pSDC, SDC_POWER_ON, 0);

}

/* Set SD_CLK Clock */
void Chip_SDC_SetClock(LPC_SDC_T *pSDC, uint32_t freq)
{
	uint32_t PClk;
	uint32_t ClkValue = 0;

	PClk = Chip_Clock_GetPeripheralClockRate();

	ClkValue = (PClk + 2 * freq - 1) / (2 * freq);
	if (ClkValue > 0) {
		ClkValue -= 1;
	}
	IP_SDC_SetClockDiv(pSDC, ClkValue);
}

/* Shutdown the SDC card controller */
void Chip_SDC_DeInit(LPC_SDC_T *pSDC)
{
	IP_SDC_DeInit(pSDC);
	Chip_Clock_DisablePeriphClock(SYSCTL_CLOCK_SDC);
}

#endif /* !defined(CHIP_LPC175X_6X) */
