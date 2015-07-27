/*
 * @brief LPC8xx FLASH Memory Controller (FMC) driver
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

#ifndef __FMC_8XX_H_
#define __FMC_8XX_H_

#ifdef __cplusplus
extern "C" {
#endif

/** @defgroup FMC_8XX CHIP: LPC8xx FLASH Memory Controller driver
 * @ingroup CHIP_8XX_Drivers
 * @{
 */

/**
 * @brief FLASH Memory Controller Unit register block structure
 */
typedef struct {
	__I  uint32_t  RESERVED1[4];
	__IO uint32_t  FLASHCFG;		/*!< Flash Configuration register */
	__I  uint32_t  RESERVED2[3];
	__IO uint32_t  FMSSTART;		/*!< Signature start address register */
	__IO uint32_t  FMSSTOP;			/*!< Signature stop address register */
	__I  uint32_t  RESERVED3;
	__I  uint32_t  FMSW0;			/*!< Signature word regsiter */
} LPC_FMC_T;

/**
 * @brief FLASH Access time definitions
 */
typedef enum {
	FLASHTIM_20MHZ_CPU = 0,		/*!< Flash accesses use 1 CPU clocks. Use for up to 20 MHz CPU clock*/
	FLASHTIM_30MHZ_CPU = 1, 	/*!< Flash accesses use 2 CPU clocks. Use for up to 30 MHz CPU clock*/
} FMC_FLASHTIM_T;

/**
 * @brief	Set FLASH memory access time in clocks
 * @param	clks	: Clock cycles for FLASH access
 * @return	Nothing
 * @note	For CPU speed up to 20MHz, use a value of 0. For up to 30MHz, use
 * a value of 1
 */
STATIC INLINE void Chip_FMC_SetFLASHAccess(FMC_FLASHTIM_T clks)
{
	uint32_t tmp = LPC_FMC->FLASHCFG & (~(0x3));

	/* Don't alter upper bits */
	LPC_FMC->FLASHCFG = tmp | clks;
}

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* __FMC_8XX_H_ */
