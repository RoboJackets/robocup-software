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

#ifndef __PMU_8XX_H_
#define __PMU_8XX_H_

#ifdef __cplusplus
extern "C" {
#endif

/** @defgroup PMU_8XX CHIP: LPC8xx PMU driver
 * @ingroup CHIP_8XX_Drivers
 * @{
 */

/**
 * @brief LPC8xx Power Management Unit register block structure
 */
typedef struct {
	__IO uint32_t PCON;		/*!< Offset: 0x000 Power control Register (R/W) */
	__IO uint32_t GPREG0;	/*!< Offset: 0x004 General purpose Register 0 (R/W) */
	__IO uint32_t GPREG1;	/*!< Offset: 0x008 General purpose Register 1 (R/W) */
	__IO uint32_t GPREG2;	/*!< Offset: 0x00C General purpose Register 2 (R/W) */
	__IO uint32_t GPREG3;	/*!< Offset: 0x010 General purpose Register 3 (R/W) */
	__IO uint32_t DPDCTRL;	/*!< Offset: 0x014 Deep power-down control register (R/W) */
} LPC_PMU_T;

/**
 * @brief LPC8xx low power mode type definitions
 */
typedef enum CHIP_PMU_MCUPOWER {
	PMU_MCU_SLEEP = 0,		/*!< Sleep mode */
	PMU_MCU_DEEP_SLEEP,		/*!< Deep Sleep mode */
	PMU_MCU_POWER_DOWN,		/*!< Power down mode */
	PMU_MCU_DEEP_PWRDOWN	/*!< Deep power down mode */
} CHIP_PMU_MCUPOWER_T;

/**
 * PMU register bit fields & masks
 */
#define PMU_PCON_PM_DPD             ((uint32_t) (1 << 0))	/*!< ARM WFI enter Deep-sleep mode */
#define PMU_PCON_PM_PWD             ((uint32_t) (1 << 1))	/*!< ARM WFI enter Power-down mode */
#define PMU_PCON_PM_DPWD            ((uint32_t) (3 << 0))	/*!< ARM WFI enter Deep Power-down mode */

#define PMU_DPDCTRL_WAKEUPPHYS      ((uint32_t) (1 << 0))	/* Wake-up pin hysteresis enable */
#define PMU_DPDCTRL_WAKEPAD         ((uint32_t) (1 << 1))	/* Wake-up pin enable */
#define PMU_DPDCTRL_LPOSCEN         ((uint32_t) (1 << 2))	/* Enable the low-power oscillator (10 khz self wk) */
#define PMU_DPDCTRL_LPOSCDPDEN      ((uint32_t) (1 << 3))	/* Enable the low-power oscillator in deep power-down*/

/**
 * @brief	Put some of the peripheral in sleep mode
 * @param	pPMU		: Pointer to PMU register block
 * @param	SleepMode	: Sleep mode
 * @return	None
 */
void Chip_PMU_Sleep(LPC_PMU_T *pPMU, CHIP_PMU_MCUPOWER_T SleepMode);

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* __PMU_8XX_H_ */
