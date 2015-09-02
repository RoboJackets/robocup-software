/*
 * @brief EEPROM driver functions
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

#include "eeprom_001.h"

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

/* Initializes EEPROM */
void IP_EEPROM_Init(IP_EEPROM_001_T *pEEPROM, uint32_t div)
{
	/* Disable EEPROM power down mode */
	IP_EEPROM_DisablePowerDown(pEEPROM);

	/* Set EEPROM clock divide value*/
	pEEPROM->CLKDIV = div;
}

/* Wait for interrupt */
void IP_EEPROM_WaitForIntStatus(IP_EEPROM_001_T *pEEPROM, uint32_t mask)
{
	uint32_t status;
	while (1) {
		status = IP_EEPROM_GetIntStatus(pEEPROM);
		if ((status & mask) == mask) {
			break;
		}
	}
	IP_EEPROM_ClearIntStatus(pEEPROM, mask);
}

/* Erase data in page register */
void IP_EEPROM_ErasePageRegister(IP_EEPROM_001_T *pEEPROM)
{
	uint32_t i = 0;

	IP_EEPROM_ClearIntStatus(pEEPROM, EEPROM_INT_ENDOFRW);

	IP_EEPROM_SetCmd(pEEPROM, EEPROM_CMD_32BITS_WRITE);

	IP_EEPROM_SetAddr(pEEPROM, 0, 0);

	for (i = 0; i < EEPROM_PAGE_SIZE; i += 4) {
		IP_EEPROM_WriteData(pEEPROM, 0);
		IP_EEPROM_WaitForIntStatus(pEEPROM, EEPROM_INT_ENDOFRW);
	}

}

/* Write data to page register */
uint32_t IP_EEPROM_WritePageRegister(IP_EEPROM_001_T *pEEPROM, uint16_t pageOffset,
									 uint8_t *pData, uint8_t wsize, uint32_t byteNum)
{
	uint32_t i = 0;
	uint32_t mask = (1 << (8 * wsize)) - 1;

	IP_EEPROM_ClearIntStatus(pEEPROM, EEPROM_INT_ENDOFRW);

	if (wsize == 1) {
		IP_EEPROM_SetCmd(pEEPROM, EEPROM_CMD_8BITS_WRITE);
	}
	else if (wsize == 2) {
		IP_EEPROM_SetCmd(pEEPROM, EEPROM_CMD_16BITS_WRITE);
	}
	else {
		IP_EEPROM_SetCmd(pEEPROM, EEPROM_CMD_32BITS_WRITE);
	}

	IP_EEPROM_SetAddr(pEEPROM, 0, pageOffset);

	for (i = 0; i < byteNum; i += wsize) {
		IP_EEPROM_WriteData(pEEPROM, (*(uint32_t *) (&pData[i])) & mask);
		IP_EEPROM_WaitForIntStatus(pEEPROM, EEPROM_INT_ENDOFRW);
	}

	return i;
}

/* Write data from page register to non-volatile memory */
void IP_EEPROM_EraseProgramPage(IP_EEPROM_001_T *pEEPROM, uint16_t pageAddr)
{
	IP_EEPROM_ClearIntStatus(pEEPROM, EEPROM_CMD_ERASE_PRG_PAGE);
	IP_EEPROM_SetAddr(pEEPROM, pageAddr, 0);
	IP_EEPROM_SetCmd(pEEPROM, EEPROM_CMD_ERASE_PRG_PAGE);
	IP_EEPROM_WaitForIntStatus(pEEPROM, EEPROM_INT_ENDOFPROG);
}

/* Read data from non-volatile memory */
uint32_t IP_EEPROM_ReadPage(IP_EEPROM_001_T *pEEPROM,
							uint16_t pageOffset,
							uint16_t pageAddr,
							uint8_t *pData,
							uint8_t rsize,
							uint32_t byteNum)
{
	uint32_t i;
	uint32_t mask = (1 << (8 * rsize)) - 1;

	IP_EEPROM_ClearIntStatus(pEEPROM, EEPROM_INT_ENDOFRW);
	IP_EEPROM_SetAddr(pEEPROM, pageAddr, pageOffset);

	if (rsize == 1) {
		IP_EEPROM_SetCmd(pEEPROM, EEPROM_CMD_8BITS_READ | EEPROM_CMD_RDPREFETCH);
	}
	else if (rsize == 2) {
		IP_EEPROM_SetCmd(pEEPROM, EEPROM_CMD_16BITS_READ | EEPROM_CMD_RDPREFETCH);
	}
	else {
		IP_EEPROM_SetCmd(pEEPROM, EEPROM_CMD_32BITS_READ | EEPROM_CMD_RDPREFETCH);
	}

	/* read and store data in buffer */
	for (i = 0; i < byteNum; i += rsize) {
		(*(uint32_t *) (&pData[i]) ) &= ~mask;
		(*(uint32_t *) (&pData[i]) ) |= (IP_EEPROM_ReadData(pEEPROM) & mask);
		IP_EEPROM_WaitForIntStatus(pEEPROM, EEPROM_INT_ENDOFRW);
	}
	return i;
}

/* Write data to EEPROM at specific address */
Status IP_EEPROM_Write(IP_EEPROM_001_T *pEEPROM,
					   uint16_t pageOffset,
					   uint16_t pageAddr,
					   void *pData,
					   IP_EEPROM_RWSIZE_T wsize,
					   uint32_t byteNum)
{
	uint32_t wTotalByteNum = 0;
	uint32_t wOffset = (pageOffset & (EEPROM_PAGE_SIZE - 1));
	uint32_t wByteNum = EEPROM_PAGE_SIZE - wOffset;
	while (byteNum) {
		if (wByteNum > byteNum) {
			wByteNum = byteNum;
		}
		/* update data to page register */
		IP_EEPROM_WritePageRegister(pEEPROM, wOffset,
									&((uint8_t *) pData)[wTotalByteNum], (uint8_t) wsize, wByteNum);
		IP_EEPROM_EraseProgramPage(pEEPROM, pageAddr);
		wTotalByteNum += wByteNum;
		byteNum -= wByteNum;

		/* Change to next page */
		pageAddr++;
		wOffset = 0;
		wByteNum = EEPROM_PAGE_SIZE;
	}
	return SUCCESS;
}

/* Read data to EEPROM at specific address */
Status IP_EEPROM_Read(IP_EEPROM_001_T *pEEPROM,
					  uint16_t pageOffset,
					  uint16_t pageAddr,
					  void *pData,
					  IP_EEPROM_RWSIZE_T rsize,
					  uint32_t byteNum)
{
	uint32_t rTotalByteNum = 0;
	uint32_t rOffset = (pageOffset & (EEPROM_PAGE_SIZE - 1));
	uint32_t rByteNum = EEPROM_PAGE_SIZE - rOffset;
	/* read and store data in buffer */
	while (byteNum) {
		if (rByteNum > byteNum) {
			rByteNum = byteNum;
		}
		/* update data to page register */
		IP_EEPROM_ReadPage(pEEPROM, rOffset, pageAddr,
						   &((uint8_t *) pData)[rTotalByteNum], (uint8_t) rsize, rByteNum);
		rTotalByteNum += rByteNum;
		byteNum -= rByteNum;

		/* Change to next page */
		pageAddr++;
		rOffset = 0;
		rByteNum = EEPROM_PAGE_SIZE;
	}
	return SUCCESS;
}

/* Erase a page at the specific address */
void IP_EEPROM_Erase(IP_EEPROM_001_T *pEEPROM, uint16_t pageAddr)
{
	IP_EEPROM_ErasePageRegister(pEEPROM);

	IP_EEPROM_EraseProgramPage(pEEPROM, pageAddr);
}
