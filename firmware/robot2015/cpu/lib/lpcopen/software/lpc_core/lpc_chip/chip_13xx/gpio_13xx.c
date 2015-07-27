/*
 * @brief LPC13xx GPIO driver
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

/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/

/*****************************************************************************
 * Private functions
 ****************************************************************************/

/*****************************************************************************
 * Public functions
 ****************************************************************************/

/* Initialize GPIO block */
void Chip_GPIO_Init(LPC_GPIO_T *pGPIO)
{
	/* Enable GPIO clocking */
	Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_GPIO);
#if defined(CHIP_LPC1347)
	IP_GPIO_Init(pGPIO);
#else
	IP_GPIO_Init(LPC_GPIO_PORTn_BASE(pGPIO, 0));
	IP_GPIO_Init(LPC_GPIO_PORTn_BASE(pGPIO, 1));
#endif
#if defined(CHIP_LPC11XXLV) || defined(CHIP_LPC11CXX)
	IP_GPIO_Init(LPC_GPIO_PORTn_BASE(pGPIO, 2));
	IP_GPIO_Init(LPC_GPIO_PORTn_BASE(pGPIO, 3));
#endif
}

/* Set Direction for a GPIO port */
void Chip_GPIO_SetDir(LPC_GPIO_T *pGPIO, uint8_t portNum, uint32_t bitValue, uint8_t out)
{
#if defined(CHIP_LPC1347)
	if (out) {
		pGPIO->DIR[portNum] |= bitValue;
	}
	else {
		pGPIO->DIR[portNum] &= ~bitValue;
	}
#else
	IP_GPIO_SetDir(LPC_GPIO_PORTn_BASE(pGPIO, portNum), bitValue, (bool) out);
#endif
}
