/*
 * @brief K9F1G driver
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

#include "board.h"

/** @defgroup K9F1G_NANDFLASH	BOARD: Driver for K9F1G
 * @ingroup BOARD_NANDFLASH
 * Various functions for reading and writing to the K9F1G NAND FLASH
 * @{
 */

/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/

/* Get column address of a page in a block */
#define COLUMN_ADDR(block, page)      (block *  K9F1G_PAGES_PER_BLOCK + page)

/* NAND information */
static const lpc_nandflash_size_t nandSize = {
	K9F1G_PAGE_SIZE,  K9F1G_SPARE_SIZE,  K9F1G_PAGES_PER_BLOCK,  K9F1G_BLOCK_COUNT
};

/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/

/*****************************************************************************
 * Private functions
 ****************************************************************************/

/*****************************************************************************
 * Public functions
 ****************************************************************************/
/* Initialize flash */
void lpc_nandflash_init(void)
{}

/* De-initialize flash */
void lpc_nandflash_DeInit(void)
{}

/* Get the flash size */
const lpc_nandflash_size_t *lpc_nandflash_get_size(void)
{
	return &nandSize;
}

/* Get flash ID */
void lpc_nandflash_get_id(uint8_t *pData)
{
	uint8_t i = 0;
#if defined(BOARD_NAND_LOCKEDCS)
	Board_NANDFLash_CSLatch(true);
#endif
	Board_NANDFLash_WriteCmd(K9F1G_READ_ID);
	Board_NANDFLash_WriteAddr(0x00);

	for (i = 0; i < sizeof(K9F1G_ID_T); i++) {
		*pData = Board_NANDFLash_ReadByte();
		pData++;
	}

#if defined(BOARD_NAND_LOCKEDCS)
	Board_NANDFLash_CSLatch(false);
#endif
}

/* Read status */
uint8_t lpc_nandflash_read_status(void)
{
	uint8_t data;

#if defined(BOARD_NAND_LOCKEDCS)
	Board_NANDFLash_CSLatch(true);
#endif
	Board_NANDFLash_WriteCmd(K9F1G_READ_STATUS);
	data = Board_NANDFLash_ReadByte();
#if defined(BOARD_NAND_LOCKEDCS)
	Board_NANDFLash_CSLatch(false);
#endif

	return data;
}

/* Erase block */
void lpc_nandflash_erase_block(uint32_t block)
{
	uint32_t row = COLUMN_ADDR(block, 0);

#if defined(BOARD_NAND_LOCKEDCS)
	Board_NANDFLash_CSLatch(true);
#endif
	Board_NANDFLash_WriteCmd(K9F1G_BLOCK_ERASE_1);

	/* Write address */
	Board_NANDFLash_WriteAddr(row & 0xFF);
	Board_NANDFLash_WriteAddr((row >> 8) & 0xFF);

	Board_NANDFLash_WriteCmd(K9F1G_BLOCK_ERASE_2);
#if defined(BOARD_NAND_LOCKEDCS)
	Board_NANDFLash_CSLatch(false);
#endif
}

/* Write buffer to flash */
uint32_t lpc_nandflash_write_page(uint32_t block, uint32_t page, uint8_t *data, uint32_t size)
{
	uint32_t i = 0;
	uint32_t row = COLUMN_ADDR(block, page);

#if defined(BOARD_NAND_LOCKEDCS)
	Board_NANDFLash_CSLatch(true);
#endif
	Board_NANDFLash_WriteCmd(K9F1G_PAGE_PROGRAM_1);

	/* Write address*/
	Board_NANDFLash_WriteAddr(0x00);
	Board_NANDFLash_WriteAddr(0x00);
	Board_NANDFLash_WriteAddr(row & 0xFF);
	Board_NANDFLash_WriteAddr((row >> 8) & 0xFF);

	/*Write data */
	for (i = 0; i < size; i++) {
		Board_NANDFLash_WriteByte(*data);
		data++;
	}

	Board_NANDFLash_WriteCmd(K9F1G_PAGE_PROGRAM_2);
#if defined(BOARD_NAND_LOCKEDCS)
	Board_NANDFLash_CSLatch(false);
#endif
	return i;
}

/* Start reading data from flash */
void lpc_nandflash_read_start(uint32_t block, uint32_t page, uint32_t ofs)
{
	uint32_t row = COLUMN_ADDR(block, page);

#if defined(BOARD_NAND_LOCKEDCS)
	Board_NANDFLash_CSLatch(true);
#endif
	Board_NANDFLash_WriteCmd(K9F1G_READ_1);

	/* Write address */
	Board_NANDFLash_WriteAddr(ofs & 0xFF);
	Board_NANDFLash_WriteAddr((ofs >> 8) & 0xFF);
	Board_NANDFLash_WriteAddr(row & 0xFF);
	Board_NANDFLash_WriteAddr((row >> 8) & 0xFF);

	Board_NANDFLash_WriteCmd(K9F1G_READ_2);
#if defined(BOARD_NAND_LOCKEDCS)
	Board_NANDFLash_CSLatch(false);
#endif
}

/* Read data from flash */
void lpc_nandflash_read_data(uint8_t *data, uint32_t size)
{
	uint32_t i = 0;

#if defined(BOARD_NAND_LOCKEDCS)
	Board_NANDFLash_CSLatch(true);
#endif
	for (i = 0; i < size; i++) {
		*data = Board_NANDFLash_ReadByte();
		data++;
	}
#if defined(BOARD_NAND_LOCKEDCS)
	Board_NANDFLash_CSLatch(false);
#endif
}

/**
 * @}
 */
