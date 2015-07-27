/*
 * @brief	USB Mass Storage data RAM module
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

/** \file
 *
 *  Header file for DataRam.c.
 */

#ifndef __RAMDISK_H_
#define __RAMDISK_H_
/* Includes: */
#include "usbmsdev.h"
#include "Descriptors.h"
#include "USB.h"

/** @defgroup EXAMPLE_DUALCORE_MassStorage LPCUSBlib Mass Storage Device Dual core example
 * @ingroup EXAMPLES_DUALCORE_43XX
 * The USB Mass Storage data RAM module module implements the functions which handles the
 * storage & retrieval of data in RAM.
 * @{
 */

/* Defines: */
/** Start address and size of RAM area which used for disk image */
	#if defined(CHIP_LPC18XX) || defined(CHIP_LPC43XX)
		#define DATA_RAM_START_ADDRESS          RAMDISK_LOCATION
		#define DATA_RAM_PHYSICAL_SIZE          RAMDISK_SIZE
		#define DATA_RAM_VIRTUAL_SIZE           0x2000
		#define DATA_RAM_BLOCK_SIZE             0x200
	#endif
	#if defined(CHIP_LPC175X_6X)
		#define DATA_RAM_START_ADDRESS          0x20080000
		#define DATA_RAM_PHYSICAL_SIZE          0x4000
		#define DATA_RAM_VIRTUAL_SIZE           0x4000
	#endif
	#if defined(CHIP_LPC177X_8X)
		#define DATA_RAM_START_ADDRESS          0x20040000
		#define DATA_RAM_PHYSICAL_SIZE          0x4000
		#define DATA_RAM_VIRTUAL_SIZE           0x4000
	#endif
	#if defined(CHIP_LPC11UXX)
		#define DATA_RAM_START_ADDRESS          0x20080000
		#define DATA_RAM_PHYSICAL_SIZE          0xa00
		#define DATA_RAM_VIRTUAL_SIZE           0x4000	/* fake capacity to trick windows */
	#endif
/** Total number of bytes of the storage medium, comprised of one or more Dataflash ICs. */
	#define VIRTUAL_MEMORY_BYTES                DATA_RAM_VIRTUAL_SIZE

/** Block size of the device. This is kept at 512 to remain compatible with the OS despite the underlying
 *  storage media (Dataflash) using a different native block size. Do not change this value.
 */
	#define VIRTUAL_MEMORY_BLOCK_SIZE           512

/** Total number of blocks of the virtual memory for reporting to the host as the device's total capacity. Do not
 *  change this value; change VIRTUAL_MEMORY_BYTES instead to alter the media size.
 */
	#define VIRTUAL_MEMORY_BLOCKS               (VIRTUAL_MEMORY_BYTES / VIRTUAL_MEMORY_BLOCK_SIZE)

/**
 * @brief	DataRAM write blocks function
 * @param   MSInterfaceInfo	: Pointer to a structure containing a Mass Storage Class configuration and state
 * @param	BlockAddress	: Data block starting address for the write sequence
 * @param	TotalBlocks	: Number of blocks of data to write
 * @return None
 * @note	Writes blocks (OS blocks, not DataRAM pages) to the storage medium, the board DataRAM IC(s), from
 * the pre-selected data OUT endpoint. This routine reads in OS sized blocks from the endpoint and writes
 * them to the DataRAM in DataRAM page sized blocks.
 */
void DataRam_WriteBlocks(USB_ClassInfo_MS_Device_t *const MSInterfaceInfo,
						 const uint32_t BlockAddress,
						 uint16_t TotalBlocks);

/**
 * @brief	DataRAM read blocks function
 * @param	MSInterfaceInfo	:	Pointer to a structure containing a Mass Storage Class configuration and state
 * @param	BlockAddress	:	Data block starting address for the read sequence
 * @param	TotalBlocks	:	Number of blocks of data to read
 *	@return None
 * @note	Reads blocks (OS blocks, not DataRAM pages) from the storage medium, the board DataRAM IC(s), into
 * the pre-selected data IN endpoint. This routine reads in DataRAM page sized blocks from the DataRAM
 * and writes them in OS sized blocks to the endpoint.
 */
void DataRam_ReadBlocks(USB_ClassInfo_MS_Device_t *const MSInterfaceInfo,
						const uint32_t BlockAddress,
						uint16_t TotalBlocks);

/**
 * @brief	DataRAM Initilisation function
 *	@return None
 * @note	This function will initialise the file system.
 */
void DataRam_Initialize(void);

/**
 * @brief	DataRAM read blocks function
 * @param	startblock	:	Starting block number
 * @param	requestblocks	:	Number of requested blocks
 * @param	availableblocks	:	Pointer to number of available
 *	@return Address of file system data
 * @note	Reads the data from file system image & return with address & number of available blocks
 */
uint32_t MassStorage_GetAddressInImage(uint32_t startblock, uint16_t requestblocks, uint16_t *availableblocks);

/**
 * @}
 */

#endif
