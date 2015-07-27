/*
 * Copyright(C) NXP Semiconductors, 2012
 * All rights reserved.
 *
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
 * Permission to use, copy, modify, and distribute this software and its
 * documentation is hereby granted, under NXP Semiconductors' and its
 * licensor's relevant copyrights in the software, without fee, provided that it
 * is used in conjunction with NXP Semiconductors microcontrollers.  This
 * copyright, permission, and disclaimer notice must appear in all copies of
 * this code.
 */

#include "acmp_001.h"

/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/

/* EDGECLR interrupt clear bit, write 1, then 0 */
#define ACMP_EDGECLR_BIT (1 << 20)

#define ACMP_EDGESEL_MASK    (0x3 << 3)
#define ACMP_COMPVPSEL_MASK  (0x7 << 8)
#define ACMP_COMPVMSEL_MASK  (0x7 << 11)
#define ACMP_HYSTERESIS_MASK (0x3 << 25)
#define ACMP_LADSEL_MASK     (0x1F << 1)
#define ACMP_LADREF_MASK     (0x1 << 6)

/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/

/*****************************************************************************
 * Private functions
 ****************************************************************************/

/*****************************************************************************
 * Public functions
 ****************************************************************************/

/* Clears the EDGECLR bit */
void IP_ACMP_EdgeClear(ACMP_001_T *pACMP)
{
	uint32_t reg = pACMP->CTRL;

	/* Toggle EDGECLR bit high and then low */
	pACMP->CTRL = reg | ACMP_EDGECLR_BIT;
	pACMP->CTRL = reg & ~ACMP_EDGECLR_BIT;
}

/* Sets up ACMP edge selection */
void IP_ACMP_SetEdgeSelection(ACMP_001_T *pACMP, IP_ACMP_001_EDGESEL_T edgeSel)
{
	uint32_t reg = pACMP->CTRL & ~ACMP_EDGESEL_MASK;

	/* Select edge for COMPEDGE */
	pACMP->CTRL = reg | (uint32_t) edgeSel;
}

/* Selects positive voltage input */
void IP_ACMP_SetPosVoltRef(ACMP_001_T *pACMP, uint32_t Posinput)
{
	uint32_t reg = pACMP->CTRL & ~ACMP_COMPVPSEL_MASK;

	/* Select positive input */
	pACMP->CTRL = reg | Posinput;
}

/* Selects negative voltage input */
void IP_ACMP_SetNegVoltRef(ACMP_001_T *pACMP, uint32_t Neginput)
{
	uint32_t reg = pACMP->CTRL & ~ACMP_COMPVMSEL_MASK;

	/* Select negative input */
	pACMP->CTRL = reg | Neginput;
}

/* Selects hysteresis level */
void IP_ACMP_SetHysteresis(ACMP_001_T *pACMP, IP_ACMP_HYS_001_T hys)
{
	uint32_t reg = pACMP->CTRL & ~ACMP_HYSTERESIS_MASK;

	/* Select negative input */
	pACMP->CTRL = reg | (uint32_t) hys;
}

/* Helper function for setting up ACMP control */
void IP_ACMP_SetupAMCPRefs(ACMP_001_T *pACMP, IP_ACMP_001_EDGESEL_T edgeSel,
						   uint32_t Posinput, uint32_t Neginput, IP_ACMP_HYS_001_T hys)
{
	uint32_t reg = pACMP->CTRL & ~(ACMP_HYSTERESIS_MASK |
								   ACMP_COMPVMSEL_MASK | ACMP_COMPVPSEL_MASK | ACMP_EDGESEL_MASK);

	/* Select negative input */
	pACMP->CTRL = reg | (uint32_t) edgeSel | (uint32_t) Posinput |
				  (uint32_t) Neginput | (uint32_t) hys;
}

/* Sets up voltage ladder */
void IP_ACMP_SetupVoltLadder(ACMP_001_T *pACMP, uint32_t ladsel, bool ladrefVDDCMP)
{
	uint32_t reg = pACMP->LAD & ~(ACMP_LADSEL_MASK | ACMP_LADREF_MASK);

	/* Setup voltage ladder and ladder reference */
	if (ladrefVDDCMP) {
		reg |= ACMP_LADREF_MASK;
	}
	pACMP->LAD = reg | ladsel;
}
