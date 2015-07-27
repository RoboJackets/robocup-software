/*
 * @brief LPC17xx/40xx SDMMC Card, ChaN FAT FS configuration file
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

#ifndef __FSMCI_CFG_H_
#define __FSMCI_CFG_H_

/**
 * @ingroup EXAMPLES_PERIPH_17XX40XX_SDC
 * @{
 */

#include <string.h>
#include "ffconf.h"
#include "diskio.h"
#include "board.h"

typedef SDMMC_CARD_T CARD_HANDLE_T;

/**
 * @def		FSMCI_CardAcquire(hc)
 * @brief	Card acquire adapter function
 * LPC17xx/40xx implementation of the FSMCI adapter function, that
 * will successfully acquire/initialize the SD Card.
 */
#define FSMCI_CardAcquire(hc)          Chip_SDMMC_Acquire(LPC_SDC, hc)

/**
 * @def		FSMCI_CardInit()
 * @brief	Initialize the card handle data structure
 * LPC17xx/40xx implementation of the FSMCI adapter function, that
 * will initialize the card handle data structure. The card handle
 * assigned by this function will be passed to all SD card adapter
 * functions.
 */
#define FSMCI_CardInit()               (&sdCardInfo)

/**
 * @def		FSMCI_CardGetSectorCnt(hc)
 * @brief	Gets number of sectors in the card
 * LPC17xx/40xx implementation of the FSMCI adapter function, that
 * will get the number of sectors in the card.
 */
#define FSMCI_CardGetSectorCnt(hc)     ((hc)->blocknr)

/**
 * @def		FSMCI_CardGetSectorSz(hc)
 * @brief	Get size of a single sector in the card
 */
#define FSMCI_CardGetSectorSz(hc)      ((hc)->block_len)


/**
 * @def		FSMCI_CardGetBlockSz(hc)
 * @brief	Get the size of one erase block in the card (Fixed to 4K)
 */
#define FSMCI_CardGetBlockSz(hc)       (4UL * 1024)

/**
 * @def		FSMCI_CardGetType(hc)
 * @brief	Get the card type
 */
#define FSMCI_CardGetType(hc)          ((hc)->card_type)

/**
 * @def		FSMCI_CardGetCSD(hc, n)
 * @brief	Get CSD data of the card at index *n*
 */
#define FSMCI_CardGetCSD(hc,n)        ((hc)->csd[(n)])

/**
 * @def		FSMCI_CardGetCID(hc, n)
 * @brief	Get CID data of the card at index *n*
 */
#define FSMCI_CardGetCID(hc,n)        ((hc)->cid[(n)])

/**
 * @def		FSMCI_CardGetState(hc,buff)
 * @brief	Read SD status into *buff*
 */
#define FSMCI_CardGetState(hc,buff) (Chip_SDMMC_GetSDStatus(LPC_SDC, hc, (uint32_t *)buff) > 0)

/**
 * @def		FSMCI_CardReadSectors(hc, buf, startSector, numSector)
 * @brief	Read data from sectors
 */
#define FSMCI_CardReadSectors(hc,buf,startSector,numSector) \
        Chip_SDMMC_ReadBlocks(LPC_SDC, hc, buf, startSector, numSector)

/**
 * @def		FSMCI_CardWriteSectors(hc, buf, startSector, numSector)
 * @brief	Write data to sectors
 */
#define FSMCI_CardWriteSectors(hc,buf,startSector,numSector) \
        Chip_SDMMC_WriteBlocks(LPC_SDC, hc, buf, startSector, numSector)

/**
 * @def		FSMCI_InitRealTimeClock()
 * @brief	Initialize the real time clock
 */
#define FSMCI_InitRealTimeClock()       rtc_initialize()

/**
 * @def		FSMCI_CardInsertWait(hc)
 * @brief	Wait for card to be inserted
 */
#define FSMCI_CardInsertWait(hc)        /* No Card detect yet */

extern CARD_HANDLE_T sdCardInfo;	/**< Type used for SD Card handle */
extern void rtc_initialize(void);   /**< RTC initialization function */

/**
 * @brief	Wait for the SD card to complete all operations and become ready
 * @param	hCrd	: Pointer to Card Handle
 * @param	tout	: Time to wait, in milliseconds
 * @return	0 when operation failed 1 when successfully completed
 */
STATIC INLINE int FSMCI_CardReadyWait(CARD_HANDLE_T *hCrd, int tout)
{
	extern volatile uint32_t timerCntms;
	uint32_t init = timerCntms;
	while (timerCntms < init + tout) {
		if (Chip_SDMMC_GetCardStatus(LPC_SDC, hCrd) & R1_READY_FOR_DATA)
			return 1;
	}
	return 0;
}

/**
 * @}
 */

#endif /* ifndef __FSMCI_CFG_H_ */
