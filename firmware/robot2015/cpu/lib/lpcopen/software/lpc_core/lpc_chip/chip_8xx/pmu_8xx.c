/*
 * @brief LPC8xx PMU chip driver
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

#define M0SCB_SLEEPONEXIT               ((uint32_t) (1 << 1))
#define M0SCB_SLEEPDEEP                 ((uint32_t) (1 << 2))
#define M0SCB_SEVONPEND                 ((uint32_t) (1 << 4))

/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/

/*****************************************************************************
 * Private functions
 ****************************************************************************/

/*****************************************************************************
 * Public functions
 ****************************************************************************/

/* Put some of the peripheral in sleep mode */
void Chip_PMU_Sleep(LPC_PMU_T *pPMU, CHIP_PMU_MCUPOWER_T SleepMode)
{
	/* Currently powered devices will not be powered down */
	Chip_SYSCTL_SetWakeup(Chip_SYSCTL_GetPowerStates());

	/* Disable powerdown for WDT and brown-out detector */
	Chip_SYSCTL_SetDeepSleepPD(~(SYSCTL_DEEPSLP_BOD_PD | SYSCTL_DEEPSLP_WDTOSC_PD));

	switch ( SleepMode ) {
	case PMU_MCU_DEEP_SLEEP:
		SCB->SCR |= M0SCB_SLEEPDEEP;
		pPMU->DPDCTRL |= PMU_DPDCTRL_LPOSCEN;
		pPMU->PCON = PMU_PCON_PM_DPD;
		break;

	case PMU_MCU_POWER_DOWN:
		SCB->SCR |= M0SCB_SLEEPDEEP;
		pPMU->DPDCTRL |= PMU_DPDCTRL_LPOSCEN;
		pPMU->PCON = PMU_PCON_PM_PWD;
		break;

	case PMU_MCU_DEEP_PWRDOWN:
		pPMU->GPREG0 = 0x12345678;
		pPMU->GPREG1 = 0x87654321;
		pPMU->GPREG2 = 0x56781234;
		pPMU->GPREG3 = 0x43218765;
		SCB->SCR |= M0SCB_SLEEPDEEP;
		pPMU->DPDCTRL |= PMU_DPDCTRL_LPOSCDPDEN;
		pPMU->PCON = PMU_PCON_PM_DPWD;
		break;

	case PMU_MCU_SLEEP:
		pPMU->DPDCTRL |= PMU_DPDCTRL_LPOSCEN;

	default:
		break;
	}
	__WFI();/* Enter to sleep mode */
}
