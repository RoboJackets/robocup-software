/*
 * @brief Common NOR Flash functions
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

#ifndef __LPC_NORFLASH_H_
#define __LPC_NORFLASH_H_

#include "board.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @defgroup BOARD_NORFLASH BOARD: Board specific NOR Flash drivers
 * @ingroup BOARD_Common
 * @{
 */

/**
 * @brief	Initialize flash
 * @return	Nothing
 */
void lpc_norflash_init(void);

/**
 * @brief	Return the flash size
 * @param	size			: pointer to where to place the total size(bytes)
 * @param	sector_count	: pointer to where to place the number of sectors
 * @return	Nothing
 */
void lpc_norflash_get_size(UNS_32 *size, UNS_32 *sector_count);

/**
 * @brief	Return the sector offset
 * @param	sector	: Sector number
 * @return	Nothing
 */
UNS_32 lpc_norflash_get_sector_offset(UNS_32 sector);

/**
 * @brief	Read manufacturer ID and device ID
 * @param	manu_id		: pointer to where to place manufacturer ID
 * @param	device_id	: pointer to where to place device ID
 * @return	Nothing
 */
void lpc_norflash_get_id(UNS_16 *manu_id, UNS_16 *device_id);

/**
 * @brief	Check Toggle Bit is being toggled or not. 
 * @param	addr	: Address
 * @return	false(being toggled)/true(stop toggling)
 * @note	During the Program/Erase operation, any consecutive attempts to read toggle bit will produce 
 * alternating "1"s and "0"s, i.e., toggling between 1 and 0. When the Program or Erase operation
* is completed, this bit will stop toggling. The flash is then ready for the next operation.<br>
 * This function reads a word at the given address (@a addr) 2 times, and then checks the toggle bit. If the value
 * of toggle bit are the same, true is returned. Otherwise, false is returned.
 */
bool lpc_norflash_toggle_bit_check(UNS_32 addr);

/**
 * @brief	Erase a sector
 * @param	addr	: Sector address
 * @return	Nothing
 */
void lpc_norflash_erase_sector(UNS_32 addr);

/**
 * @brief	Write data to flash
 * @param	addr	: Address
 * @param	data	: Data value to write
 * @return	Nothing
 * @note	addr must be word-aligned.
 */
void lpc_norflash_write_word(UNS_32 addr, UNS_16 data);

/**
 * @brief	Write buffer to flash
 * @param	addr	: Address
 * @param	data	: Pointer to data to write
 * @param	size	The number of  (bytes)
 * @return	The number of written bytes
 * @note	addr must be word-aligned.
 */
UNS_32 lpc_norflash_write_buffer(UNS_32 addr, UNS_16 *data, UNS_32 size);

/**
 * @brief	Read data from flash
 * @param	addr	: Address
 * @return	Data value read from the address
 * @note	addr must be word-aligned.
 */
UNS_16 lpc_norflash_read_word(UNS_32 addr);

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* __LPC_NORFLASH_H_ */
