/*
 * @brief Common NAND Flash functions
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

#ifndef __LPC_NANDFLASH_H_
#define __LPC_NANDFLASH_H_

#include "board.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @defgroup BOARD_NANDFLASH BOARD: Board specific NAND Flash drivers
 * @ingroup BOARD_Common
 * @{
 */

/**
 * @brief	NAND Flash Size structure
 */
typedef struct {
	UNS_32  page_size;				/*!< Page Size */
	UNS_32  spare_size;				/*!< Spare Size */
	UNS_32  pages_per_block;		/*!< The number of pages per block */
	UNS_32  block_cnt;				/*!< The number of blocks */
} lpc_nandflash_size_t;

/**
 * @brief	Initialize flash
 * @return	Nothing
 */
void lpc_nandflash_init(void);

/**
 * @brief	De-initialize flash
 * @return	Nothing
 */
void lpc_nandflash_DeInit(void);

/**
 * @brief	Return the flash size
 * @return	NAND FLASH information
 */
const lpc_nandflash_size_t *lpc_nandflash_get_size(void);

/**
 * @brief	Read manufacturer ID and device ID
 * @param	pData	pointer to buffer to read
 * @return	Nothing
 */
void lpc_nandflash_get_id(uint8_t *pData);

/**
 * @brief	Read status
 * @return	status byte (or-ed bit value of NANDFLASH_STATUS_*)
 */
uint8_t lpc_nandflash_read_status(void);

/**
 * @brief	Erase a block
 * @param	block	: block address
 * @return	Nothing
 * @note	After returning from this function, read the status to get the result.
 */
void lpc_nandflash_erase_block(uint32_t block);

/**
 * @brief	Write a page to NAND FLASH
 * @param	block	: block index
 * @param	page	: page index
 * @param	data	: pointer to buffer to write
 * @param	size	: the number of written bytes
 * @return	The number of written bytes
 * @note	After returning from this function, read the status to get the result.
 */
uint32_t lpc_nandflash_write_page(uint32_t block, uint32_t page, uint8_t *data, uint32_t size);

/**
 * @brief	Start reading data from NAND FLASH
 * @param	block	: block index
 * @param	page	: page index
 * @param	ofs		: offset in page
 * @return	Nothing
 */
void lpc_nandflash_read_start(uint32_t block, uint32_t page, uint32_t ofs);

/**
 * @brief	Read data from NAND FLASH
 * @param	data	: pointer to buffer to read
 * @param	size	: the number of read bytes
 * @return	Nothing
 */
void lpc_nandflash_read_data(uint8_t *data, uint32_t size);

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* __LPC_NANDFLASH_H_ */
