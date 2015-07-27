/*
 * @brief SDMMC Chan FATFS simple abstraction layer
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

#include "fsmci_cfg.h"
#include "board.h"
#include "chip.h"

/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/

/* Disk Status */
static volatile DSTATUS Stat = STA_NOINIT;

/* 100Hz decrement timer stopped at zero (disk_timerproc()) */
static volatile WORD Timer2;

static CARD_HANDLE_T *hCard;

/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/

/*****************************************************************************
 * Private functions
 ****************************************************************************/

/*****************************************************************************
 * Public functions
 ****************************************************************************/

/* Initialize Disk Drive */
DSTATUS disk_initialize(BYTE drv)
{
	if (drv) {
		return STA_NOINIT;				/* Supports only single drive */
	}
	/*	if (Stat & STA_NODISK) return Stat;	*//* No card in the socket */

	if (Stat != STA_NOINIT) {
		return Stat;					/* card is already enumerated */

	}

	#if !_FS_READONLY
	FSMCI_InitRealTimeClock();
	#endif

	/* Initialize the Card Data Strucutre */
	hCard = FSMCI_CardInit();

	/* Reset */
	Stat = STA_NOINIT;

	FSMCI_CardInsertWait(hCard); /* Wait for card to be inserted */

	/* Enumerate the card once detected. Note this function may block for a little while. */
	if (!FSMCI_CardAcquire(hCard)) {
		DEBUGOUT("Card Acquire failed...\r\n");
		return Stat;
	}

	Stat &= ~STA_NOINIT;
	return Stat;

}

/* Disk Drive miscellaneous Functions */
DRESULT disk_ioctl(BYTE drv, BYTE ctrl, void *buff)
{
	DRESULT res;
	BYTE *ptr = buff;

	if (drv) {
		return RES_PARERR;
	}
	if (Stat & STA_NOINIT) {
		return RES_NOTRDY;
	}

	res = RES_ERROR;

	switch (ctrl) {
	case CTRL_SYNC:	/* Make sure that no pending write process */
		if (FSMCI_CardReadyWait(hCard, 50)) {
			res = RES_OK;
		}
		break;

	case GET_SECTOR_COUNT:	/* Get number of sectors on the disk (DWORD) */
		*(DWORD *) buff = FSMCI_CardGetSectorCnt(hCard);
		res = RES_OK;
		break;

	case GET_SECTOR_SIZE:	/* Get R/W sector size (WORD) */
		*(WORD *) buff = FSMCI_CardGetSectorSz(hCard);
		res = RES_OK;
		break;

	case GET_BLOCK_SIZE:/* Get erase block size in unit of sector (DWORD) */
		*(DWORD *) buff = FSMCI_CardGetBlockSz(hCard);
		res = RES_OK;
		break;

	case MMC_GET_TYPE:		/* Get card type flags (1 byte) */
		*ptr = FSMCI_CardGetType(hCard);
		res = RES_OK;
		break;

	case MMC_GET_CSD:		/* Receive CSD as a data block (16 bytes) */
		*((uint32_t *) buff + 0) = FSMCI_CardGetCSD(hCard, 0);
		*((uint32_t *) buff + 1) = FSMCI_CardGetCSD(hCard, 1);
		*((uint32_t *) buff + 2) = FSMCI_CardGetCSD(hCard, 2);
		*((uint32_t *) buff + 3) = FSMCI_CardGetCSD(hCard, 3);
		res = RES_OK;
		break;

	case MMC_GET_CID:		/* Receive CID as a data block (16 bytes) */
		*((uint32_t *) buff + 0) = FSMCI_CardGetCID(hCard, 0);
		*((uint32_t *) buff + 1) = FSMCI_CardGetCID(hCard, 1);
		*((uint32_t *) buff + 2) = FSMCI_CardGetCID(hCard, 2);
		*((uint32_t *) buff + 3) = FSMCI_CardGetCID(hCard, 3);
		res = RES_OK;
		break;

	case MMC_GET_SDSTAT:/* Receive SD status as a data block (64 bytes) */
		if (FSMCI_CardGetState(hCard, (uint8_t *) buff)) {
			res = RES_OK;
		}
	break;

	default:
		res = RES_PARERR;
		break;
	}

	return res;
}

/* Read Sector(s) */
DRESULT disk_read(BYTE drv, BYTE *buff, DWORD sector, BYTE count)
{
	if (drv || !count) {
		return RES_PARERR;
	}
	if (Stat & STA_NOINIT) {
		return RES_NOTRDY;
	}

	if (FSMCI_CardReadSectors(hCard, buff, sector, count)) {
		return RES_OK;
	}

	return RES_ERROR;
}

/* Get Disk Status */
DSTATUS disk_status(BYTE drv)
{
	if (drv) {
		return STA_NOINIT;	/* Supports only single drive */

	}
	return Stat;
}

/* Write Sector(s) */
DRESULT disk_write(BYTE drv, const BYTE *buff, DWORD sector, BYTE count)
{

	if (drv || !count) {
		return RES_PARERR;
	}
	if (Stat & STA_NOINIT) {
		return RES_NOTRDY;
	}

	if ( FSMCI_CardWriteSectors(hCard, (void *) buff, sector, count)) {
		return RES_OK;
	}

	return RES_ERROR;
}
