/*
 * @brief K9F1G NAND Flash definitions
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

#ifndef __LPC_K9F1G_NANDFLASH_H_
#define __LPC_K9F1G_NANDFLASH_H_

#include "board.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @defgroup BOARD_K9F1G_NANDFLASH BOARD: K9F1G NAND Flash drivers
 * @ingroup BOARD_NANDFLASH
 * @{
 */

#define KF91G_NANDFLASH

/* Page size */
#define K9F1G_PAGE_SIZE              (2 << 10)		/* 2K bytes */
/* Spare start address */
#define K9F1G_SPARE_START_ADDR       ( K9F1G_PAGE_SIZE)	/* Spare start address */
/* Spare size */
#define K9F1G_SPARE_SIZE             (64)
/* Pages per block */
#define K9F1G_PAGES_PER_BLOCK        (64)
/* The number of blocks */
#define K9F1G_BLOCK_COUNT            (1024)

/*
 * @brief	K9F1G commands
 */
#define K9F1G_READ_1                  0x00
#define K9F1G_READ_2                  0x30
#define K9F1G_READ_ID                 0x90
#define K9F1G_RESET                   0xFF
#define K9F1G_PAGE_PROGRAM_1          0x80
#define K9F1G_PAGE_PROGRAM_2          0x10
#define K9F1G_BLOCK_ERASE_1           0x60
#define K9F1G_BLOCK_ERASE_2           0xD0
#define K9F1G_READ_STATUS             0x70

/*
 * @brief	K9F1G ID structures
 */
typedef struct {
	uint8_t MarkerCode;
	uint8_t DeviceCode;
	struct {
		uint8_t InternalChipNum : 2;		/** Internal Chip Number. 0: 1, 1:2, 2:4, 3:8 */
		uint8_t CellType : 2;				/** Cell Type. 0: 2 Level Cell, 1: 4 Level Cell, 2: 8 Level Cell, 3: 16 Level Cell */
		uint8_t MaxProgramPageNum : 2;		/** Number of Simultaneously Programmed Pages. 0: 1, 1:2, 2:4, 3:8 */
		uint8_t InterProgSupport : 1;		/** Interleave Program Between multiple chips. 0: Not Support, 1: Support */
		uint8_t CacheProgSupport : 1;		/** Cache Program. 0: Not Support, 1: Support */
	} b3;

	struct {
		uint8_t PageSize : 2;				/**Page Sizer. 0: 1KB, 1:2KB, 2:4KB, 3:8KB */
		uint8_t BlockSize : 2;				/**Block Sizer. 0: 64KB, 1:128KB, 2:256KB, 3:512KB */
		uint8_t RedundantSize : 1;			/**Redundant Area Size ( byte/512byte). 0: 8, 1: 16*/
		uint8_t Organization : 1;			/**Organization. 0: x8, 1: x16*/
		uint8_t SeriaAccessMin : 2;			/**Serial Access Minimum. 0: 50ns/30ns, 1: 25ns, 2 & 3: Reserved*/
	} b4;

	struct {
		uint8_t Reserved0 : 2;				/** Reserved */
		uint8_t PlaneNum : 2;				/** Plane Number. 0: 1, 1:2, 2:4, 3:8 */
		uint8_t PlaneSize : 3;				/** Plane Size. Size = 64KB << PlaneSize value */
		uint8_t Reserved1 : 1;				/** Reserved */
	} b5;

} K9F1G_ID_T;

/*
 * @brief NAND FLASH status
 */
#define NANDFLASH_STATUS_PAGE_PROG_FAIL         (1 << 0)
#define NANDFLASH_STATUS_BLOCK_ERASE_FAIL       (1 << 0)
#define NANDFLASH_STATUS_DEV_READY              (1 << 6)
#define NANDFLASH_STATUS_READ_WRITE_NPROTECTED  (1 << 7)

/*
 * @brief	Timming defitnitions (in ns)
 */
/** tPROG */
#define NANDFLASH_PROG_TIME                     (700 * 1000)
/** tBERS */
#define NANDFLASH_BLOCK_ERASE_TIME              (3 * 1000 * 1000)
/** tR */
#define NANDFLASH_READ_TIME                     (25 * 1000)
/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* __LPC_K9F1G_NANDFLASH_H_ */
