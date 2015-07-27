/*
 * @brief LPC13xx Flash/EEPROM programming
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

#ifndef __FLASH_EEPROM_13XX_H_
#define __FLASH_EEPROM_13XX_H_

#if defined(CHIP_LPC1347)

#ifdef __cplusplus
extern "C" {
#endif

/** @defgroup FLASH_EEPROM_13XX CHIP: LPC13xx FLASH/EEPROM Programming
 * @ingroup CHIP_13XX_Drivers
 * @{
 */

/** Flash Programming Entry Location */
#define FLASH_ENTRY_LOCATION        (0x1fff1ff1)

/** EEPROM size */
#define EEPROM_SIZE                 (4 << 10)

/** Part ID definitions */
#define PART_ID                     (0x08020543)

/**
 * @brief	Get sector number of the specified address
 * @param	adr		: flash address.
 * @return	sector number
 */
STATIC INLINE uint32_t Chip_FLASH_GetSecNum(uint32_t adr)
{
	return adr >> 12;
}

/**
 * @brief	Execute flash programming command
 * @param	pCommand	: Command information
 * @param	pOutput	: Output information
 * @return	Nothing
 */
STATIC INLINE void Chip_FLASH_Execute(FLASH_COMMAND_T *pCommand, FLASH_OUTPUT_T *pOutput)
{
	IP_FLASH_Execute(((FLASH_ENTRY_T) FLASH_ENTRY_LOCATION), pCommand, pOutput);
}

/**
 * @brief	Prepare sector(s) for write operation
 * @param	pCommand	: Command information
 * @param	pOutput	: Output information
 * @return	Nothing
 * @note	This command must be executed before executing "Copy RAM to flash" or "Erase Sector(s)" command. <br>
 *			The boot sector can not be prepared by this command.
 */
STATIC INLINE void Chip_FLASH_PrepareSectors(FLASH_PREPARE_SECTORS_COMMAND_T *pCommand,
											 FLASH_PREPARE_SECTORS_OUTPUT_T *pOutput)
{
	Chip_FLASH_Execute((FLASH_COMMAND_T *) pCommand, (FLASH_OUTPUT_T *) pOutput);
}

/**
 * @brief	Copy RAM to flash
 * @param	pCommand	: Command information
 * @param	pOutput	: Output information
 * @return	Nothing
 * @note	The affected sectors should be prepared first by calling "Prepare Sector for Write Operation" command.<br>
 *		The boot sector can not be written by this command.
 */
STATIC INLINE void Chip_FLASH_CopyRamToFlash(FLASH_COPY_RAM_TO_FLASH_COMMAND_T *pCommand,
											 FLASH_COPY_RAM_TO_FLASH_OUTPUT_T *pOutput)
{
	Chip_FLASH_Execute((FLASH_COMMAND_T *) pCommand, (FLASH_OUTPUT_T *) pOutput);
}

/**
 * @brief	Erase Sector(s)
 * @param	pCommand	: Command information
 * @param	pOutput	: Output information
 * @return	Nothing
 * @note	 The boot sector can not be erased by this command.
 */
STATIC INLINE void Chip_FLASH_EraseSectors(FLASH_ERASE_SECTORS_COMMAND_T *pCommand,
										   FLASH_ERASE_SECTORS_OUTPUT_T *pOutput)
{
	Chip_FLASH_Execute((FLASH_COMMAND_T *) pCommand, (FLASH_OUTPUT_T *) pOutput);
}

/**
 * @brief	Blank check sector(s)
 * @param	pCommand	: Command information
 * @param	pOutput	: Output information
 * @return	Nothing
 */
STATIC INLINE void Chip_FLASH_BlankCheckSectors(FLASH_BLANK_CHECK_SECTORS_COMMAND_T *pCommand,
												FLASH_BLANK_CHECK_SECTORS_OUTPUT_T *pOutput)
{
	Chip_FLASH_Execute((FLASH_COMMAND_T *) pCommand, (FLASH_OUTPUT_T *) pOutput);
}

/**
 * @brief	Read Part Identification number
 * @param	pOutput	: Output information
 * @return	Nothing
 */
void Chip_FLASH_ReadPartID(FLASH_READ_PART_ID_OUTPUT_T *pOutput);

/**
 * @brief	Read Boot code version number
 * @param	pOutput	: Output information
 * @return	Nothing
 */
void Chip_FLASH_ReadBootCodeVersion(FLASH_READ_BOOTCODE_VER_OUTPUT_T *pOutput);

/**
 * @brief	Compare memory
 * @param	pCommand	: parameters
 * @param	pOutput	: results
 * @return	Nothing
 * @note	The result may not be correct when the source or destination includes any
 * of the first 512 bytes starting from address zero. The first 512 bytes can be re-mapped to RAM.
 */
STATIC INLINE void Chip_FLASH_CompareMem(FLASH_COMPARE_MEM_COMMAND_T *pCommand,
										 FLASH_COMPARE_MEM_OUTPUT_T *pOutput)
{
	Chip_FLASH_Execute((FLASH_COMMAND_T *) pCommand, (FLASH_OUTPUT_T *) pOutput);
}

/**
 * @brief	Reinvoke ISP
 * @return	Nothing
 */
void Chip_FLASH_ReInvokeISP(void);

/**
 * @brief	Read UID
 * @param	pOutput	: Output information
 * @return	Nothing
 */
void Chip_FLASH_ReadUID(FLASH_READ_UID_OUTPUT_T *pOutput);
/**
 * @brief	Erase Page(s)
 * @param	pCommand	: Command information
 * @param	pOutput	: Output information
 * @return	Nothing
 */
STATIC INLINE void Chip_FLASH_ErasePages(FLASH_ERASE_PAGES_COMMAND_T *pCommand,
										 FLASH_ERASE_PAGES_OUTPUT_T *pOutput)
{
	Chip_FLASH_Execute((FLASH_COMMAND_T *) pCommand, (FLASH_OUTPUT_T *) pOutput);
}

/**
 * @brief	Write EEPROM
 * @param	pCommand	: Command information
 * @param	pOutput	: Output information
 * @return	Nothing
 * @note		The top 64 bytes of the EEPROM memory are reserved and cannot be written to.
 */
STATIC INLINE void Chip_EEPROM_Write(EEPROM_WRITE_COMMAND_T *pCommand,
									 EEPROM_WRITE_OUTPUT_T *pOutput)
{
	Chip_FLASH_Execute((FLASH_COMMAND_T *) pCommand, (FLASH_OUTPUT_T *) pOutput);
}

/**
 * @brief	Read EEPROM
 * @param	pCommand	: Command information
 * @param	pOutput	: Output information
 * @return	Nothing
 */
STATIC INLINE void Chip_EEPROM_Read(EEPROM_READ_COMMAND_T *pCommand,
									EEPROM_READ_OUTPUT_T *pOutput)
{
	Chip_FLASH_Execute((FLASH_COMMAND_T *) pCommand, (FLASH_OUTPUT_T *) pOutput);
}

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* defined(CHIP_LPC1347) */

#endif /* __FLASH_EEPROM_13XX_H_ */
