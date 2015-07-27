/*
 * @brief LPC8xx Pin Interrupt and Pattern Match Registers and driver
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

#ifndef __PIN_INT_8XX_H_
#define __PIN_INT_8XX_H_

#ifdef __cplusplus
extern "C" {
#endif

/** @defgroup PININT_8XX CHIP: LPC8xx Pin Interrupt and Pattern Match driver
 * @ingroup CHIP_8XX_Drivers
 * @{
 */

/**
 * @brief LPC8xx Pin Interrupt and Pattern Match register block structure
 */
typedef struct {							/*!< (@ 0xA0004000) PIN_INT Structure */
	__IO uint32_t ISEL;						/*!< (@ 0xA0004000) Pin Interrupt Mode register */
	__IO uint32_t IENR;						/*!< (@ 0xA0004004) Pin Interrupt Enable (Rising) register */
	__IO uint32_t SIENR;					/*!< (@ 0xA0004008) Set Pin Interrupt Enable (Rising) register */
	__IO uint32_t CIENR;					/*!< (@ 0xA000400C) Clear Pin Interrupt Enable (Rising) register */
	__IO uint32_t IENF;						/*!< (@ 0xA0004010) Pin Interrupt Enable Falling Edge / Active Level register */
	__IO uint32_t SIENF;					/*!< (@ 0xA0004014) Set Pin Interrupt Enable Falling Edge / Active Level register */
	__IO uint32_t CIENF;					/*!< (@ 0xA0004018) Clear Pin Interrupt Enable Falling Edge / Active Level address */
	__IO uint32_t RISE;						/*!< (@ 0xA000401C) Pin Interrupt Rising Edge register */
	__IO uint32_t FALL;						/*!< (@ 0xA0004020) Pin Interrupt Falling Edge register */
	__IO uint32_t IST;						/*!< (@ 0xA0004024) Pin Interrupt Status register */
	__IO uint32_t PMCTRL;					/*!< (@ 0xA0004028) GPIO pattern match interrupt control register          */
	__IO uint32_t PMSRC;					/*!< (@ 0xA000402C) GPIO pattern match interrupt bit-slice source register */
	__IO uint32_t PMCFG;					/*!< (@ 0xA0004030) GPIO pattern match interrupt bit slice configuration register */
} LPC_PIN_INT_T;

/**
 * LPC8xx Pin Interrupt and Pattern match engine register
 * bit fields and macros
 */

#define PMODE_EDGE                  (0)	/*!< Edge sensitive interrupt */
#define PMODE_LEVEL                 (1)	/*!< Level sensitive interrupt */
/*!< Set Pin Interrupt mode */
#define PMODE_SET(t, p, x)          (x ? (t->ISEL |= (x << p)) : (t->ISEL &= ~(1 << p)))

#define ENRL_DISABLE                (0)	/*!< Disable raising edge or level interrupt */
#define ENRL_ENABLE                 (1)	/*!< Enable raising edge or level inetrrupt */
/*!< Enable or disable raising edge or level interrupt */
#define ENRL_SET(t, p, x)           (x ? (t->IENR |= (x << p)) : (t->IENR &= ~(1 << p)))

#define SETENRL(t, p)               (t->SIENR |= (1 << p))	/*!< Enable raising edge or level inetrrupt */
#define CENRL(t, p)                 (t->CIENR |= (1 << p))	/*!< Disable raising edge or level inetrrupt */

#define ENAF_LOW                    (0)	/*!< Disable falling edge or active level low interrupt */
#define ENAF_HIGH                   (1)	/*!< Enable falling edge or active level low interrupt */
/*!< Enable or disable falling edge or active level low interrupt */
#define ENAF_SET(t, p, x)           (x ? (t->IENF |= (x << p)) : (t->IENF &= ~(1 << p)))

#define SETENAF(t, p)               (t->SIENF |= (1 << p))	/*!< Enable falling edge or active level low interrupt */
#define CENAF(t, p)                 (t->CIENF |= (1 << p))	/*!< Disable falling edge or active level low interrupt */

/*!< Clear Raising Edge detection */
#define RDET_CLR(t, p)              (t->RISE |= (1 << p))
/*!< Clear Falling Edge detection */
#define FDET_CLR(t, p)              (t->FALL |= (1 << p))
/*!< Clear Interrupt status */
#define PSTAT_CRL(t, p)             (t->IST |= (1 << p))

/**
 * LPC8xx Pin Matching Interrupt pins enum
 */
typedef enum CHIP_PMI_PIN {
	PMI0 = 0,	/*!< PMI PIN 0 */
	PMI1 = 1,	/*!< PMI PIN 1 */
	PMI2 = 2,	/*!< PMI PIN 2 */
	PMI3 = 3,	/*!< PMI PIN 3 */
	PMI4 = 4,	/*!< PMI PIN 4 */
	PMI5 = 5,	/*!< PMI PIN 5 */
	PMI6 = 6,	/*!< PMI PIN 6 */
	PMI7 = 7	/*!< PMI PIN 7 */
} CHIP_PMI_PIN_T;

/**
 * LPC8xx Pin Interrupt Configuration values enum
 */
typedef enum CHIP_PINT_CFG {
	PINT_ACTIVE_LOW = 0,	/*!< Active Low */
	PINT_ACTIVE_HIGH = 1,	/*!< Active High */
	PINT_EDGE_DET = 2,		/*!< Edge Detect */
	PINT_LEVEL_DET = 4,		/*!< Level detect */
	PINT_RISING = 8,		/*!< Raising edge */
	PINT_FALLING = 16		/*!< Falling edge */
} CHIP_PINT_CFG_T;

/**
 * @brief	Configure the Pattern Matching pins
 * @param   pin     : Pin Number
 * @param   cfg     : Pin configuration
 * @return	Nothing
 */
void Chip_PININT_Config(CHIP_PMI_PIN_T pin, CHIP_PINT_CFG_T cfg);

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* __PIN_INT_8XX_H_ */
