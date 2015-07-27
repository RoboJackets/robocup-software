/*
 * @brief Power Management Unit registers and control functions
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

#ifndef __PMU_11XX_H_
#define __PMU_11XX_H_

#ifdef __cplusplus
extern "C" {
#endif

/** @defgroup PMU_11XX CHIP: LPC11xx Power Management Unit block driver
 * @ingroup CHIP_11XX_Drivers
 * @{
 */

/**
 * @brief Power Management Unit register block structure
 */
typedef struct {				/*!< PMU Structure          */
	__IO uint32_t  PCON;		/*!< Power control register */
	__IO uint32_t  GPREG[4];	/*!< Power GP register      */
} LPC_PMU_T;

/**
 * @brief	PMU Set mode
 * @param	pPMU	: The base of PMU peripheral on the chip
 * @param	pm		: PMU mode
 * @return	Nothing
 */
STATIC INLINE void Chip_PMU_SetMode(LPC_PMU_T *pPMU, uint8_t pm)
{
	pPMU->PCON = pm & 0x3;
}

/**
 * @brief	PMU Get data
 * @param	pPMU	: The base of PMU peripheral on the chip
 * @param	reg		: PMU register number
 * @return	Contents of PMU register
 */
STATIC INLINE uint32_t Chip_PMU_GetData(LPC_PMU_T *pPMU, uint8_t reg)
{
	return pPMU->GPREG[reg];
}

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* __PMU_11XX_H_ */
