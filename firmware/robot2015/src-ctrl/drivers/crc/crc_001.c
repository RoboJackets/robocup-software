/*
 * @brief Cyclic Redundancy Check (CRC) Engine driver
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

#include "crc_001.h"

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

/* Sets up the CRC engine with defaults based on the polynomial to be used */
void IP_CRC_UseDefaultConfig(IP_CRC_001_T *pCRC, IP_CRC_001_POLY_T poly)
{
	switch (poly) {
	case CRC_POLY_CRC16:
		pCRC->MODE = MODE_CFG_CRC16;
		pCRC->SEED = CRC_SEED_CRC16;
		break;

	case CRC_POLY_CRC32:
		pCRC->MODE = MODE_CFG_CRC32;
		pCRC->SEED = CRC_SEED_CRC32;
		break;

	case CRC_POLY_CCITT:
	default:
		pCRC->MODE = MODE_CFG_CCITT;
		pCRC->SEED = CRC_SEED_CCITT;
		break;
	}
}

/* Convenience function for computing a standard CCITT checksum from an 8-bit data block */
uint32_t IP_CRC_CRC8(IP_CRC_001_T *pCRC, const uint8_t *data, uint32_t bytes)
{
	while (bytes > 0) {
		IP_CRC_Write8(pCRC, *data);
		data++;
		bytes--;
	}
	return IP_CRC_ReadSum(pCRC);
}

/* Configure CRC engine and compute CRC16 checksum from 16-bit data */
uint32_t IP_CRC_CRC16(IP_CRC_001_T *pCRC, const uint16_t *data, uint32_t hwords)
{
	while (hwords > 0) {
		IP_CRC_Write16(pCRC, *data);
		data++;
		hwords--;
	}
	return IP_CRC_ReadSum(pCRC);
}

/* Configure CRC engine and compute CRC32 checksum from 32-bit data */
uint32_t IP_CRC_CRC32(IP_CRC_001_T *pCRC, const uint32_t *data, uint32_t words)
{
	while (words > 0) {
		IP_CRC_Write32(pCRC, *data);
		data++;
		words--;
	}
	return IP_CRC_ReadSum(pCRC);
}
