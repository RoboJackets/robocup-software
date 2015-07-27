/*
 * @brief LPC11xx analog comparator driver
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

#ifndef __ACMP_11XX_H_
#define __ACMP_11XX_H_

#if defined(CHIP_LPC11AXX)

#ifdef __cplusplus
extern "C" {
#endif

/** @defgroup ACMP11XX CHIP: LPC11xx ACMP Driver
 * @ingroup CHIP_11XX_Drivers
 * This driver provides Analog comparator driver support for the device.
 * The driver requires the following IP drivers:<br>
 * @ref IP_ACMP_001<br>
 * @{
 */

/**
 * Analog Comparator positive input values
 */
typedef enum CHIP_ACMP_POS_INPUT {
	ACMP_POSIN_VLO      = (0 << 8),	/*!< Voltage ladder output */
	ACMP_POSIN_ACMP_I1  = (1 << 8),	/*!< ACMP_I1 pin */
	ACMP_POSIN_ACMP_I2  = (2 << 8),	/*!< ACMP_I2 pin */
	ACMP_POSIN_ACMP_I3  = (3 << 8),	/*!< ACMP_I3 pin */
	ACMP_POSIN_ACMP_I4  = (4 << 8),	/*!< ACMP_I4 pin */
	ACMP_POSIN_ACMP_I5  = (5 << 8),	/*!< ACMP_I5 pin */
	ACMP_POSIN_INT_REF  = (6 << 8),	/*!< Internal reference voltage */
	ACMP_POSIN_TEMPSENS = (7 << 8)	/*!< Temperature sensor */
} CHIP_ACMP_POS_INPUT_T;

/**
 * Analog Comparator negative input values
 */
typedef enum CHIP_ACMP_NEG_INPUT {
	ACMP_NEGIN_VLO     = (0 << 11),	/*!< Voltage ladder output */
	ACMP_NEGIN_ACMP_I1 = (1 << 11),	/*!< ACMP_I1 pin */
	ACMP_NEGIN_ACMP_I2 = (2 << 11),	/*!< ACMP_I2 pin */
	ACMP_NEGIN_ACMP_I3 = (3 << 11),	/*!< ACMP_I3 pin */
	ACMP_NEGIN_ACMP_I4 = (4 << 11),	/*!< ACMP_I4 pin */
	ACMP_NEGIN_ACMP_I5 = (5 << 11),	/*!< ACMP_I5 pin */
	ACMP_NEGIN_INT_REF = (6 << 11)	/*!< Internal reference voltage */
} CHIP_ACMP_NEG_INPUT_T;

/**
 * @brief	Initializes the ACMP
 * @param	pACMP	: Pointer to Analog Comparator block
 * @return	Nothing
 */
void Chip_ACMP_Init(LPC_CMP_T *pACMP);

/**
 * @brief	Deinitializes the ACMP
 * @param	pACMP	: Pointer to Analog Comparator block
 * @return	Nothing
 */
void Chip_ACMP_Deinit(LPC_CMP_T *pACMP);

/**
 * @brief	Returns the current comparator status
 * @param	pACMP	: Pointer to Analog Comparator block
 * @return	Status is an Or'ed value of ACMP_COMPSTAT_BIT or ACMP_COMPEDGE_BIT
 */
STATIC INLINE uint32_t Chip_ACMP_GetCompStatus(LPC_CMP_T *pACMP)
{
	return IP_ACMP_GetCompStatus(pACMP);
}

/**
 * @brief	Clears the ACMP interrupt (EDGECLR bit)
 * @param	pACMP	: Pointer to Analog Comparator block
 * @return	Nothing
 */
STATIC INLINE void Chip_ACMP_EdgeClear(LPC_CMP_T *pACMP)
{
	IP_ACMP_EdgeClear(pACMP);
}

/**
 * @brief	Sets up ACMP edge selection
 * @param	pACMP	: Pointer to Analog Comparator block
 * @param	edgeSel	: Edge selection value
 * @return	Nothing
 */
STATIC INLINE void Chip_ACMP_SetEdgeSelection(LPC_CMP_T *pACMP, IP_ACMP_001_EDGESEL_T edgeSel)
{
	IP_ACMP_SetEdgeSelection(pACMP, edgeSel);
}

/**
 * @brief	Synchronizes Comparator output to bus clock
 * @param	pACMP	: Pointer to Analog Comparator block
 * @return	Nothing
 */
STATIC INLINE void Chip_ACMP_EnableSyncCompOut(LPC_CMP_T *pACMP)
{
	IP_ACMP_EnableSyncCompOut(pACMP);
}

/**
 * @brief	Sets comparator output to be used directly (no sync)
 * @param	pACMP	: Pointer to Analog Comparator block
 * @return	Nothing
 */
STATIC INLINE void Chip_ACMP_DisableSyncCompOut(LPC_CMP_T *pACMP)
{
	IP_ACMP_DisableSyncCompOut(pACMP);
}

/**
 * @brief	Selects positive voltage input
 * @param	pACMP		: Pointer to Analog Comparator block
 * @param	Posinput	: one of the positive input voltage sources
 * @return	Nothing
 */
STATIC INLINE void Chip_ACMP_SetPosVoltRef(LPC_CMP_T *pACMP, CHIP_ACMP_POS_INPUT_T Posinput)
{
	IP_ACMP_SetPosVoltRef(pACMP, (uint32_t) Posinput);
}

/**
 * @brief	Selects negative voltage input
 * @param	pACMP		: Pointer to Analog Comparator block
 * @param	Neginput	: one of the negative input voltage sources
 * @return	Nothing
 */
STATIC INLINE void Chip_ACMP_SetNegVoltRef(LPC_CMP_T *pACMP, CHIP_ACMP_NEG_INPUT_T Neginput)
{
	IP_ACMP_SetNegVoltRef(pACMP, (uint32_t) Neginput);
}

/**
 * @brief	Selects hysteresis level
 * @param	pACMP	: Pointer to Analog Comparator block
 * @param   hys     : Selected Hysteresis level
 * @return	Nothing
 */
STATIC INLINE void Chip_ACMP_SetHysteresis(LPC_CMP_T *pACMP, IP_ACMP_HYS_001_T hys)
{
	IP_ACMP_SetHysteresis(pACMP, hys);
}

/**
 * @brief	Helper function for setting up ACMP control
 * @param	pACMP		: Pointer to Analog Comparator block
 * @param	edgeSel		: Edge selection value
 * @param	Posinput	: one of the positive input voltage sources
 * @param	Neginput	: one of the negative input voltage sources
 * @param	hys			: Selected Hysteresis level
 * @return	Nothing
 */
STATIC INLINE void Chip_ACMP_SetupAMCPRefs(LPC_CMP_T *pACMP, IP_ACMP_001_EDGESEL_T edgeSel,
										   CHIP_ACMP_POS_INPUT_T Posinput, CHIP_ACMP_NEG_INPUT_T Neginput,
										   IP_ACMP_HYS_001_T hys)
{
	IP_ACMP_SetupAMCPRefs(pACMP, edgeSel, (uint32_t) Posinput,
						  (uint32_t) Neginput, hys);
}

/**
 * @brief	Sets up voltage ladder
 * @param	pACMP			: Pointer to Analog Comparator block
 * @param	ladsel			: Voltage ladder value (0 .. 31)
 * @param	ladrefVDDCMP	: Selects the reference voltage Vref for the voltage ladder
 *							: false for VDD, true for VDDCMP pin
 * @return	Nothing
 */
STATIC INLINE void Chip_ACMP_SetupVoltLadder(LPC_CMP_T *pACMP, uint32_t ladsel, bool ladrefVDDCMP)
{
	IP_ACMP_SetupVoltLadder(pACMP, (ladsel << 1), ladrefVDDCMP);
}

/**
 * @brief	Enables voltage ladder
 * @param	pACMP	: Pointer to Analog Comparator block
 * @return	Nothing
 */
STATIC INLINE void Chip_ACMP_EnableVoltLadder(LPC_CMP_T *pACMP)
{
	IP_ACMP_EnableVoltLadder(pACMP);
}

/**
 * @brief	Disables voltage ladder
 * @param	pACMP	: Pointer to Analog Comparator block
 * @return	Nothing
 */
STATIC INLINE void Chip_ACMP_DisableVoltLadder(LPC_CMP_T *pACMP)
{
	IP_ACMP_DisableVoltLadder(pACMP);
}

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* defined(CHIP_LPC11AXX) */

#endif /* __ACMP_11XX_H_ */
