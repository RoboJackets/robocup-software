/*
 * @brief LPC8xx State Configurable Timer (SCT) Chip driver
 *
 * @note
 * Copyright(C) NXP Semiconductors, 2012
 * All rights reserved.
 *
 * @par
 * Software that is described herein is for illustrative purposes only
 * which provides customers with programming information regarding the
 * LPC products.  This software is supplied "AS IS" without any warranties of
 * any kind, and NXP Semiconductors and its licenser disclaim any and
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

#ifndef __SCT_8XX_H_
#define __SCT_8XX_H_

#ifdef __cplusplus
extern "C" {
#endif

/** @defgroup SCT_8XX CHIP: LPC8xx State Configurable Timer (SCT) driver
 * @ingroup CHIP_8XX_Drivers
 * This driver provides State Configurable Timer (SCT) support for the device.
 * The driver requires the following IP drivers:<br>
 * @ref IP_SCT_001<br>
 * @{
 */

/**
 * SCT Match register values enum
 */
typedef enum CHIP_SCT_MATCH_REG {
	SCT_MATCH_0 = 0,	/*!< SCT Match register 0 */
	SCT_MATCH_1 = 1,	/*!< SCT Match register 1 */
	SCT_MATCH_2 = 2,	/*!< SCT Match register 2 */
	SCT_MATCH_3 = 3,	/*!< SCT Match register 3 */
	SCT_MATCH_4 = 4		/*!< SCT Match register 4 */
} CHIP_SCT_MATCH_REG_T;

/**
 * SCT Event values enum
 */
typedef enum CHIP_SCT_EVENT {
	SCT_EVT_0  = (1 << 0),	/*!< Event 0 */
	SCT_EVT_1  = (1 << 1),	/*!< Event 1 */
	SCT_EVT_2  = (1 << 2),	/*!< Event 2 */
	SCT_EVT_3  = (1 << 3),	/*!< Event 3 */
	SCT_EVT_4  = (1 << 4)	/*!< Event 4 */
} CHIP_SCT_EVENT_T;

/**
 * @brief	Configures the State Configurable Timer
 * @param	pSCT	: The base of SCT peripheral on the chip
 * @param	value	: The 32-bit CONFIG register value
 * @return	Nothing
 */
STATIC INLINE void Chip_SCT_Config(LPC_SCT_T *pSCT, uint32_t value)
{
	IP_SCT_Config(pSCT, value);
}

/**
 * @brief	Set unified count value in State Configurable Timer
 * @param	pSCT	: The base of SCT peripheral on the chip
 * @param	count	: The 32-bit count value
 * @return	Nothing
 */
STATIC INLINE void Chip_SCT_SetCount(LPC_SCT_T *pSCT, uint32_t count)
{
	IP_SCT_SetCount(pSCT, count);
}

/**
 * @brief	Set lower count value in State Configurable Timer
 * @param	pSCT	: The base of SCT peripheral on the chip
 * @param	count	: The 16-bit count value
 * @return	Nothing
 */
STATIC INLINE void Chip_SCT_SetCountL(LPC_SCT_T *pSCT, uint16_t count)
{
	IP_SCT_SetCountL(pSCT, count);
}

/**
 * @brief	Set higher count value in State Configurable Timer
 * @param	pSCT	: The base of SCT peripheral on the chip
 * @param	count	: The 16-bit count value
 * @return	Nothing
 */
STATIC INLINE void Chip_SCT_SetCountH(LPC_SCT_T *pSCT, uint16_t count)
{
	IP_SCT_SetCountH(pSCT, count);
}

/**
 * @brief	Set unified match count value in State Configurable Timer
 * @param	pSCT	: The base of SCT peripheral on the chip
 * @param	n		: Match register value
 * @param	value	: The 32-bit match count value
 * @return	Nothing
 */
STATIC INLINE void Chip_SCT_SetMatchCount(LPC_SCT_T *pSCT, CHIP_SCT_MATCH_REG_T n, uint32_t value)
{
	IP_SCT_SetMatchCount(pSCT, (uint32_t) n, value);
}

/**
 * @brief	Set unified match reload count value in State Configurable Timer
 * @param	pSCT	: The base of SCT peripheral on the chip
 * @param	n		: Match register value
 * @param	value	: The 32-bit match count reload value
 * @return	Nothing
 */
STATIC INLINE void Chip_SCT_SetMatchReload(LPC_SCT_T *pSCT, CHIP_SCT_MATCH_REG_T n, uint32_t value)
{
	IP_SCT_SetMatchReload(pSCT, (uint32_t) n, value);
}

/**
 * @brief	Enable the interrupt for the specified event in State Configurable Timer
 * @param	pSCT	: The base of SCT peripheral on the chip
 * @param	evt		: Event value
 * @return	Nothing
 */
STATIC INLINE void Chip_SCT_EventIntEnable(LPC_SCT_T *pSCT, CHIP_SCT_EVENT_T evt)
{
	IP_SCT_EventIntEnable(pSCT, (uint32_t) evt);
}

/**
 * @brief	Disable the interrupt for the specified event in State Configurable Timer
 * @param	pSCT	: The base of SCT peripheral on the chip
 * @param	evt		: Event value
 * @return	Nothing
 */
STATIC INLINE void Chip_SCT_EventIntDisable(LPC_SCT_T *pSCT, CHIP_SCT_EVENT_T evt)
{
	IP_SCT_EventIntDisable(pSCT, (uint32_t) evt);
}

/**
 * @brief	Clear the specified event flag in State Configurable Timer
 * @param	pSCT	: The base of SCT peripheral on the chip
 * @param	evt		: Event value
 * @return	Nothing
 */
STATIC INLINE void Chip_SCT_ClearEventFlag(LPC_SCT_T *pSCT, CHIP_SCT_EVENT_T evt)
{
	IP_SCT_ClearEventFlag(pSCT, (uint32_t) evt);
}

/**
 * @brief	Initializes the State Configurable Timer
 * @param	pSCT	: The base of SCT peripheral on the chip
 * @return	Nothing
 */
void Chip_SCT_Init(LPC_SCT_T *pSCT);

/**
 * @brief	Deinitializes the State Configurable Timer
 * @param	pSCT	: The base of SCT peripheral on the chip
 * @return	Nothing
 */
void Chip_SCT_DeInit(LPC_SCT_T *pSCT);

/**
 * @}
 */

#ifdef __cplusplus
}

#endif

#endif /* __SCT_8XX_H_ */
