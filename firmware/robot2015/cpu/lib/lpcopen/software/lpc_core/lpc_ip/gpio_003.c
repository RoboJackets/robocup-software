/*
 * @brief GPIO Registers and Functions
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

#include "gpio_003.h"

#if (defined(CHIP_LPC110X) || defined(CHIP_LPC11XXLV) || defined(CHIP_LPC11CXX) || defined(CHIP_LPC1343))

/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/

/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/

/*****************************************************************************
 * Private functions
 ****************************************************************************/

STATIC INLINE void configGpioInt(IP_GPIO_003_T *pGPIO, uint8_t pin, bool ibe, bool isense, bool iev)
{
	uint32_t temp;

	temp = pGPIO->IBE & ~(1 << pin);
	pGPIO->IBE = temp | (ibe << pin);
	temp = pGPIO->IS & ~(1 << pin);
	pGPIO->IS = temp | (isense << pin);
	temp = pGPIO->IEV & ~(1 << pin);
	pGPIO->IEV = temp | (iev << pin);
}

/*****************************************************************************
 * Public functions
 ****************************************************************************/

/* Enable GPIO Interrupt */
void IP_GPIO_IntCmd(IP_GPIO_003_T *pGPIO, uint8_t pin, IP_GPIOPININT_MODE_T mode)
{
	configGpioInt(pGPIO, pin, (bool) ((mode >> 31) & 1), (bool) (mode & 1), (bool) ((mode >> 16) & 1));
	pGPIO->IE |= 1 << pin;
}

#endif
