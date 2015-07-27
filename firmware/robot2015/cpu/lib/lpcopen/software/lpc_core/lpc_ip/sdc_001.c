/*
 * @brief	SD Card Interface registers and control functions
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

#include "sdc_001.h"

/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/

static void writeDelay(void)
{
	volatile uint8_t i;
	for ( i = 0; i < 0x10; i++ ) {	/* delay 3MCLK + 2PCLK  */
	}
}

/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/

/*****************************************************************************
 * Private functions
 ****************************************************************************/

/*****************************************************************************
 * Public functions
 ****************************************************************************/

/* Set power state of SDC peripheral */
void IP_SDC_PowerControl(IP_SDC_001_T *pSDC, IP_SDC_001_PWR_CTRL_T pwrMode, uint32_t flag)
{
	pSDC->POWER = SDC_PWR_CTRL(pwrMode) | flag;
	writeDelay();
}

/* Set clock divider for SDC peripheral */
void IP_SDC_SetClockDiv(IP_SDC_001_T *pSDC, uint8_t div)
{
	uint32_t temp;
	temp = (pSDC->CLOCK & (~SDC_CLOCK_CLKDIV_BITMASK));
	pSDC->CLOCK = temp | (SDC_CLOCK_CLKDIV(div));
	writeDelay();
}

/* Clock control for SDC peripheral*/
void IP_SDC_ClockControl(IP_SDC_001_T *pSDC, IP_SDC_001_CLOCK_CTRL_T ctrlType,
						 FunctionalState NewState)
{
	if (NewState) {
		pSDC->CLOCK |= (1 << ctrlType);
	}
	else {
		pSDC->CLOCK &= (~(1 << ctrlType));
	}
	writeDelay();
}

/* Initialize SDC peripheral */
void IP_SDC_Init(IP_SDC_001_T *pSDC)
{
	/* Disable SD_CLK */
	IP_SDC_ClockControl(pSDC, SDC_CLOCK_ENABLE, DISABLE);

	/* Power-off */
	IP_SDC_PowerControl(pSDC, SDC_POWER_OFF, 0);
	writeDelay();

	/* Disable all interrupts */
	pSDC->MASK0 = 0;

	/*Setting for timeout problem */
	pSDC->DATATIMER = 0x1FFFFFFF;

	pSDC->COMMAND = 0;
	writeDelay();

	pSDC->DATACTRL = 0;
	writeDelay();

    /* clear all pending interrupts */
	pSDC->CLEAR = SDC_CLEAR_ALL;		
}

/* Set Command Info */
void IP_SDC_SetCommand(IP_SDC_001_T *pSDC, uint32_t Cmd, uint32_t Arg)
{
    /* Clear status register */
	pSDC->CLEAR = SDC_CLEAR_ALL;

    /* Set the argument first, finally command */
	pSDC->ARGUMENT = Arg;

    /* Write command value, enable the command */
	pSDC->COMMAND = Cmd | SDC_COMMAND_ENABLE;

	writeDelay();
}

/* Reset Command Info */
void IP_SDC_ResetCommand(IP_SDC_001_T *pSDC)
{
	pSDC->CLEAR = SDC_CLEAR_ALL;

	pSDC->ARGUMENT = 0xFFFFFFFF;

	pSDC->COMMAND = 0;

	writeDelay();
}

/* Get Command response */
void IP_SDC_GetResp(IP_SDC_001_T *pSDC, IP_SDC_001_RESP_T *pResp)
{
	uint8_t i;
	pResp->CmdIndex = SDC_RESPCOMMAND_VAL(pSDC->RESPCMD);
	for (i = 0; i < SDC_CARDSTATUS_BYTENUM; i++) {
		pResp->Data[i] = pSDC->RESPONSE[i];
	}
}

/* Setup Data Transfer Information */
void IP_SDC_SetDataTransfer(IP_SDC_001_T *pSDC, IP_SDC_001_DATA_TRANSFER_T *pTransfer)
{
	uint32_t DataCtrl = 0;
	pSDC->DATATIMER = pTransfer->Timeout;
	pSDC->DATALENGTH = pTransfer->BlockNum * SDC_DATACTRL_BLOCKSIZE_VAL(pTransfer->BlockSize);
	DataCtrl = SDC_DATACTRL_ENABLE;
	DataCtrl |= ((uint32_t)pTransfer->Dir) | ((uint32_t)pTransfer->Mode) | SDC_DATACTRL_BLOCKSIZE(pTransfer->BlockSize);
	if (pTransfer->DMAUsed) {
		DataCtrl |= SDC_DATACTRL_DMA_ENABLE;
	}
	pSDC->DATACTRL = DataCtrl;
	writeDelay();
}

/* Write data to FIFO */
void IP_SDC_WriteFIFO(IP_SDC_001_T *pSDC, uint32_t *pSrc, bool bFirstHalf)
{
	uint8_t start = 0, end = 7;
	if (!bFirstHalf) {
		start += 8;
		end += 8;
	}
	for (; start <= end; start++) {
		pSDC->FIFO[start] = *pSrc;
		pSrc++;
	}
}

/* Read data from FIFO */
void IP_SDC_ReadFIFO(IP_SDC_001_T *pSDC, uint32_t *pDst, bool bFirstHalf)
{
	uint8_t start = 0, end = 7;

	if (!bFirstHalf) {
		start += 8;
		end += 8;
	}
	for (; start <= end; start++) {
		*pDst = pSDC->FIFO[start];
		pDst++;
	}
}

