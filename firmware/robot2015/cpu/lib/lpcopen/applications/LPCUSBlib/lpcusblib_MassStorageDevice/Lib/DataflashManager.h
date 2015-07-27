/*
 * @brief Header file for DataflashManager.c
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

/** @file
 *
 *  Header file for DataflashManager.c.
 */

#ifndef __DATAFLASH_MANAGER_H_
#define __DATAFLASH_MANAGER_H_

#include "../MassStorage.h"
#include "../Descriptors.h"
#include "USB.h"

*Preprocessor Checks : * /
#if (DATAFLASH_PAGE_SIZE % 16)
	#error Dataflash page size must be a multiple of 16 bytes.
#endif

/** Total number of bytes of the storage medium, comprised of one or more Dataflash ICs. */
#define VIRTUAL_MEMORY_BYTES                ((uint32_t) DATAFLASH_PAGES * DATAFLASH_PAGE_SIZE * DATAFLASH_TOTALCHIPS)

/** Block size of the device. This is kept at 512 to remain compatible with the OS despite the underlying
 *  storage media (Dataflash) using a different native block size. Do not change this value.
 */
#define VIRTUAL_MEMORY_BLOCK_SIZE           512

/** Total number of blocks of the virtual memory for reporting to the host as the device's total capacity. Do not
 *  change this value; change VIRTUAL_MEMORY_BYTES instead to alter the media size.
 */
#define VIRTUAL_MEMORY_BLOCKS               (VIRTUAL_MEMORY_BYTES / VIRTUAL_MEMORY_BLOCK_SIZE)

void DataflashManager_WriteBlocks(USB_ClassInfo_MS_Device_t *const MSInterfaceInfo,
								  const uint32_t BlockAddress,
								  uint16_t TotalBlocks);

void DataflashManager_ReadBlocks(USB_ClassInfo_MS_Device_t *const MSInterfaceInfo,
								 const uint32_t BlockAddress,
								 uint16_t TotalBlocks);

void DataflashManager_WriteBlocks_RAM(const uint32_t BlockAddress,
									  uint16_t TotalBlocks,
									  uint8_t *BufferPtr) ATTR_NON_NULL_PTR_ARG(3);

void DataflashManager_ReadBlocks_RAM(const uint32_t BlockAddress,
									 uint16_t TotalBlocks,
									 uint8_t *BufferPtr) ATTR_NON_NULL_PTR_ARG(3);

void DataflashManager_ResetDataflashProtections(void);

bool DataflashManager_CheckDataflashOperation(void);

#endif /* __DATAFLASH_MANAGER_H_ */
