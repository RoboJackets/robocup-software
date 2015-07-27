/*
 * @brief CAN Registers and control functions
 *
 * @note
 * Copyright(C) NXP Semiconductors, 2012
 * All rights reserved.
 *
 * @par
 * Software that is described herein is for illustrative purposes only
 * which provides customers with programming information regarding the
 * LPC products. This software is supplied "AS IS" without any warranties of
 * any kind, and NXP Semiconductors and its licensor disclaim any and
 * all warranties, express or implied, including all implied warranties of
 * merchantability, fitness for a particular purpose and non-infringement of
 * intellectual property rights. NXP Semiconductors assumes no responsibility
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
 * is used in conjunction with NXP Semiconductors microcontrollers. This
 * copyright, permission, and disclaimer notice must appear in all copies of
 * this code.
 */

#include "can_001.h"

/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/

/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/

/*****************************************************************************
 * Private functions
 ****************************************************************************/

/* Get start row, end row of the given section */
STATIC void getSectionAddress(IP_CAN_001_AF_T *pCanAF,
							  IP_CAN_AF_RAM_SECTION_T SectionID, uint16_t *StartAddr, uint16_t *EndAddr)
{
	if (SectionID == CANAF_RAM_FULLCAN_SEC) {
		*StartAddr = 0;
	}
	else {
		*StartAddr = CANAF_ENDADDR_VAL(pCanAF->ENDADDR[SectionID - 1]);
	}
	*EndAddr = CANAF_ENDADDR_VAL(pCanAF->ENDADDR[SectionID]);

	if (*EndAddr > 0) {
		*EndAddr -= 1;	/* Minus 1 to get the actual end row */
	}
	else {
		*EndAddr = *StartAddr;
	}
}

/* Get the total number of entries in LUT */
STATIC INLINE uint16_t getTotalEntryNum(IP_CAN_001_AF_T *pCanAF)
{
	return CANAF_ENDADDR_VAL(pCanAF->ENDADDR[CANAF_RAM_EFF_GRP_SEC]);	/* Extended ID Group section is the last section of LUT */
}

/* Set the End Address of a section. EndAddr = the number of the last row of the section + 1. */
STATIC INLINE void setSectionEndAddress(IP_CAN_001_AF_T *pCanAF,
										IP_CAN_AF_RAM_SECTION_T SectionID, uint16_t EndAddr)
{
	pCanAF->ENDADDR[SectionID] = CANAF_ENDADDR(EndAddr);
}

/* Get information of the received frame. Return ERROR means no message came.*/
STATIC INLINE Status getReceiveFrameInfo(IP_CAN_001_T *pCAN,
										 IP_CAN_001_RX_T *pRxFrame)
{
	*pRxFrame = pCAN->RX;
	return (pCAN->SR & CAN_SR_RBS(0)) ? SUCCESS : ERROR;
}

/* Set Tx Frame Information */
STATIC INLINE void setSendFrameInfo(IP_CAN_001_T *pCAN, IP_CAN_BUFFER_ID_T TxBufID,
									IP_CAN_001_TX_T *pTxFrame)
{
	pCAN->TX[TxBufID] = *pTxFrame;
}

/* Create the standard ID entry */
STATIC uint16_t createStdIDEntry(IP_CAN_STD_ID_Entry_T *pEntryInfo, bool IsFullCANEntry)
{
	uint16_t Entry = 0;
	Entry = (pEntryInfo->CtrlNo & CAN_STD_ENTRY_CTRL_NO_MASK) << CAN_STD_ENTRY_CTRL_NO_POS;
	Entry |= (pEntryInfo->Disable & CAN_STD_ENTRY_DISABLE_MASK) << CAN_STD_ENTRY_DISABLE_POS;
	Entry |= (pEntryInfo->ID_11 & CAN_STD_ENTRY_ID_MASK) << CAN_STD_ENTRY_ID_POS;
	if (IsFullCANEntry) {
		Entry |= 1 << CAN_STD_ENTRY_IE_POS;
	}
	return Entry;
}

STATIC INLINE uint16_t createUnUsedSTDEntry(uint8_t CtrlNo)
{
	return ((CtrlNo & CAN_STD_ENTRY_CTRL_NO_MASK) << CAN_STD_ENTRY_CTRL_NO_POS) | (1 << CAN_STD_ENTRY_DISABLE_POS);
}

/* Get information from the standard ID entry */
STATIC void readStdIDEntry(uint16_t EntryVal, IP_CAN_STD_ID_Entry_T *pEntryInfo)
{
	pEntryInfo->CtrlNo = (EntryVal >> CAN_STD_ENTRY_CTRL_NO_POS) & CAN_STD_ENTRY_CTRL_NO_MASK;
	pEntryInfo->Disable = (EntryVal >> CAN_STD_ENTRY_DISABLE_POS) & CAN_STD_ENTRY_DISABLE_MASK;
	pEntryInfo->ID_11 = (EntryVal >> CAN_STD_ENTRY_ID_POS) & CAN_STD_ENTRY_ID_MASK;
}

/* Setup Standard iD section */
STATIC Status setupSTDSection(uint32_t *pCanAFRamAddr,
							  IP_CAN_STD_ID_Entry_T *pStdCANSec,
							  uint16_t EntryNum,
							  bool IsFullCANEntry)
{
	uint16_t i;
	uint16_t CurID = 0;
	uint16_t Entry;
	uint16_t EntryCnt = 0;

	/* Setup FullCAN section */
	for (i = 0; i < EntryNum; i += 2) {
		/* First Entry */
		if (CurID > pStdCANSec[i].ID_11) {
			return ERROR;
		}
		CurID = pStdCANSec[i].ID_11;
		Entry = createStdIDEntry(&pStdCANSec[i], IsFullCANEntry);
		pCanAFRamAddr[EntryCnt] = Entry << 16;

		/* Second Entry */
		if ((i + 1) < EntryNum) {
			if (CurID > pStdCANSec[i + 1].ID_11) {
				return ERROR;
			}
			CurID = pStdCANSec[i + 1].ID_11;
			Entry = createStdIDEntry(&pStdCANSec[i + 1], IsFullCANEntry);
			pCanAFRamAddr[EntryCnt] |= Entry;
		}
		else {
			pCanAFRamAddr[EntryCnt] |= createUnUsedSTDEntry(pStdCANSec[0].CtrlNo);
		}
		EntryCnt++;
	}
	return SUCCESS;

}

/* Setup the Group Standard ID section */
STATIC Status setupSTDRangeSection(uint32_t *pCanAFRamAddr,
								   IP_CAN_STD_ID_RANGE_Entry_T *pStdRangeCANSec,
								   uint16_t EntryNum)
{
	return setupSTDSection(pCanAFRamAddr, (IP_CAN_STD_ID_Entry_T *) pStdRangeCANSec, EntryNum * 2, false);
}

/* Shift a number of entries down 1 position */
STATIC void shiftSTDEntryDown(uint32_t *arr, int32_t num)
{
	uint32_t i = 0;
	uint32_t prevRow, curRow;

	if (num <= 0) {
		return;
	}

	prevRow = arr[0];
	arr[0] = ((prevRow & 0xFFFF0000) >> 16);
	for (i = 0; i < (num / 2); i++) {
		curRow = arr[i + 1];
		arr[i + 1] = ((prevRow & 0xFFFF) << 16) | ((curRow & 0xFFFF0000) >> 16);
		prevRow = curRow;
	}
	if ((num % 2) == 0) {
		uint8_t CtrlNo = (prevRow >> CAN_STD_ENTRY_CTRL_NO_POS) & CAN_STD_ENTRY_CTRL_NO_MASK;
		arr[num / 2] = ((prevRow & 0xFFFF) << 16) | createUnUsedSTDEntry(CtrlNo);
	}
}

/* Shift a number of entries up 1 position */
STATIC void shiftSTDEntryUp(uint32_t *arr, int32_t num)
{
	int32_t i = 0;
	uint32_t prevRow = 0, curRow = 0;
	uint8_t CtrlNo = 0;

	if (num <= 0) {
		return;
	}

	curRow = arr[((num + 1) / 2) - 1];
	CtrlNo = (curRow >> CAN_STD_ENTRY_CTRL_NO_POS) & CAN_STD_ENTRY_CTRL_NO_MASK;

	/* If num is odd, the last row only includes one item. Therefore, there is nothing
	   to do with it. If num is even, shift the last item to one place of the row.*/
	if ((num % 2) == 0) {
		arr[((num + 1) / 2) - 1] = ((curRow & 0xFFFF) << 16) | createUnUsedSTDEntry(CtrlNo);
	}
	/* Shift from the row before the last row */
	for (i = ((num + 1) / 2) - 1; i > 0; i--) {
		prevRow = arr[i - 1];
		arr[i - 1] = ((curRow & 0xFFFF0000) >> 16) | ((prevRow & 0xFFFF) << 16);
		curRow = prevRow;
	}
}

/* Create an extended ID entry */
STATIC uint32_t createExtIDEntry(IP_CAN_EXT_ID_Entry_T *pEntryInfo)
{
	uint32_t Entry = 0;
	Entry = (pEntryInfo->CtrlNo & CAN_EXT_ENTRY_CTRL_NO_MASK) << CAN_EXT_ENTRY_CTRL_NO_POS;
	Entry |= (pEntryInfo->ID_29 & CAN_EXT_ENTRY_ID_MASK) << CAN_EXT_ENTRY_ID_POS;
	return Entry;
}

/* Get information from an extended ID entry */
STATIC void readExtIDEntry(uint32_t EntryVal, IP_CAN_EXT_ID_Entry_T *pEntryInfo)
{
	pEntryInfo->CtrlNo = (EntryVal >> CAN_EXT_ENTRY_CTRL_NO_POS) & CAN_EXT_ENTRY_CTRL_NO_MASK;
	pEntryInfo->ID_29 = (EntryVal >> CAN_EXT_ENTRY_ID_POS) & CAN_EXT_ENTRY_ID_MASK;
}

/* Setup the Extended ID Section */
STATIC Status setupEXTSection(uint32_t *pCanAFRamAddr, IP_CAN_EXT_ID_Entry_T *pExtCANSec, uint16_t EntryNum)
{
	uint16_t i;
	uint32_t CurID = 0;
	uint32_t Entry;
	uint16_t EntryCnt = 0;

	/* Setup FullCAN section */
	for (i = 0; i < EntryNum; i++) {
		if (CurID > pExtCANSec[i].ID_29) {
			return ERROR;
		}
		CurID = pExtCANSec[i].ID_29;
		Entry = createExtIDEntry(&pExtCANSec[i]);
		pCanAFRamAddr[EntryCnt] = Entry;
		EntryCnt++;
	}
	return SUCCESS;

}

/* Setup Group Extended ID section */
STATIC Status setupEXTRangeSection(uint32_t *pCanAFRamAddr,
								   IP_CAN_EXT_ID_RANGE_Entry_T *pExtRangeCANSec,
								   uint16_t EntryNum)
{
	return setupEXTSection(pCanAFRamAddr, (IP_CAN_EXT_ID_Entry_T *) pExtRangeCANSec, EntryNum * 2);
}

/* Get entry value from the given start index of the given array. byteNum is 2 or 4 bytes */
STATIC uint32_t getArrayVal(uint8_t *arr, uint32_t startIndex, uint8_t byteNum)
{
	uint8_t i = 0;
	uint32_t retVal = 0;
	uint32_t index;

	index =  startIndex * byteNum;
	if (byteNum == 2) {	// each entry uses 2 bytes
		if (startIndex % 2) {
			index -= 2;		// little endian
		}
		else {
			index += 2;
		}
	}

	for (i = 0; i < byteNum; i++) {
		retVal |= arr[index + i] << (8 * i);
	}
	return retVal;
}

/* Search the index to insert the new entry */
STATIC int32_t searchInsertIndex(uint32_t *arr, uint32_t arrNum, uint32_t val, uint32_t mask, uint8_t unitSize)
{
	uint32_t LowerIndex, UpperIndex, MidIndex;
	uint32_t MidVal;
	uint8_t *byteArray = (uint8_t *) arr;

	if (arrNum == 0) {
		return 0;		/* Insert into the first line */

	}
	LowerIndex = 0;
	UpperIndex = arrNum - 1;
	while (LowerIndex + 1 < UpperIndex) {
		MidIndex = (LowerIndex + UpperIndex) / 2;
		MidVal = getArrayVal(byteArray, MidIndex, unitSize) & mask;
		if (MidVal == val) {
			return -1;			/* The new value is already in the array */
		}
		else if (MidVal < val) {
			LowerIndex = MidIndex + 1;
		}
		else {
			UpperIndex = MidIndex - 1;
		}
	}

	if (((getArrayVal(byteArray, LowerIndex, unitSize) & mask) == val) ||
		((getArrayVal(byteArray, UpperIndex, unitSize) & mask) == val)) {
		return -1;			/* The new value is already in the array */
	}
	if ((getArrayVal(byteArray, LowerIndex, unitSize) & mask) > val) {
		return LowerIndex;
	}
	if ((getArrayVal(byteArray, UpperIndex, unitSize) & mask) < val) {
		return UpperIndex + 1;
	}

	return UpperIndex;
}

/*****************************************************************************
 * Public functions
 ****************************************************************************/

/* Initialize the the RTC peripheral */
void IP_CAN_Init(IP_CAN_001_T *pCAN) {
	volatile uint32_t i;
	/* Enter to Reset Mode */
	pCAN->MOD = CAN_MOD_RM;

	/* Disable all CAN Interrupts */
	pCAN->IER &= (~CAN_IER_BITMASK) & CAN_IER_BITMASK;
	pCAN->GSR &= (~CAN_GSR_BITMASK) & CAN_GSR_BITMASK;

	/* Request command to release Rx, Tx buffer and clear data overrun */
	pCAN->CMR = CAN_CMR_RRB | CAN_CMR_AT | CAN_CMR_CDO;

	/* Read to clear interrupt pending in interrupt capture register */
	i = pCAN->ICR;

	/* Return to normal mode */
	pCAN->MOD = CAN_MOD_OPERATION;
}

/* Set CAN Mode */
void IP_CAN_SetMode(IP_CAN_001_T *pCAN, uint32_t Mode, FunctionalState NewState)
{
	if ((Mode & CAN_MOD_LOM) || (Mode & CAN_MOD_STM)) {
		/* Enter to Reset Mode */
		pCAN->MOD |= CAN_MOD_RM;

		/* Change to the given mode */
		if (NewState) {
			pCAN->MOD |= Mode;
		}
		else {
			pCAN->MOD &= (~Mode) & CAN_MOD_BITMASK;
		}

		/* Release Reset Mode */
		pCAN->MOD &= (~CAN_MOD_RM) & CAN_MOD_BITMASK;
	}
	else {
		if (NewState) {
			pCAN->MOD |= Mode;
		}
		else {
			pCAN->MOD &= (~Mode) & CAN_MOD_BITMASK;
		}
	}

}

/* Set Bus Timing */
void IP_CAN_SetBusTiming(IP_CAN_001_T *pCAN, IP_CAN_BUS_TIMING_T *pBusTiming) {
	/* Enter to Reset Mode */
	IP_CAN_SetMode(pCAN, CAN_MOD_RM, ENABLE);

	/* Set Bus Timing */
	pCAN->BTR = CAN_BTR_BRP(pBusTiming->BRP)
				| CAN_BTR_SJW(pBusTiming->SJW)
				| CAN_BTR_TESG1(pBusTiming->TESG1)
				| CAN_BTR_TESG2(pBusTiming->TESG2);

	if (pBusTiming->SAM) {
		pCAN->BTR |= CAN_BTR_SAM;
	}

	/* Exit from Reset Mode */
	IP_CAN_SetMode(pCAN, CAN_MOD_RM, DISABLE);
}

/* Receive CAN Message */
Status IP_CAN_Receive(IP_CAN_001_T *pCAN, IP_CAN_MSG_T *pMsg) {
	int8_t i;
	IP_CAN_001_RX_T RxFrame;
	if (getReceiveFrameInfo(pCAN, &RxFrame) == SUCCESS) {

		/* Read Message Identifier */
		if (RxFrame.RFS & CAN_RFS_FF) {
			pMsg->ID = CAN_EXTEND_ID_USAGE | CAN_RID_ID_29(RxFrame.RID);
		}
		else {
			pMsg->ID = CAN_RID_ID_11(RxFrame.RID);
		}

		/* Read Data Length */
		pMsg->DLC = CAN_RFS_DLC(RxFrame.RFS);

		/* Read Message Type */
		pMsg->Type = 0;
		if (RxFrame.RFS & CAN_RFS_RTR) {
			pMsg->Type |= CAN_REMOTE_MSG;
		}
		else {
			/* Read data only if the received message is not Remote message */
			for (i = 0; i < CAN_MSG_MAX_DATA_LEN; i++) {
				pMsg->Data[i] = (RxFrame.RD[i / 4] >> (8 * (i % 4))) & 0xFF;
			}
		}

		/* Release received message */
		IP_CAN_SetCmd(pCAN, CAN_CMR_RRB);

		return SUCCESS;
	}
	return ERROR;
}

/* Send CAN Message */
Status IP_CAN_Send(IP_CAN_001_T *pCAN, IP_CAN_BUFFER_ID_T TxBufID, IP_CAN_MSG_T *pMsg)
{
	uint8_t i = 0;
	IP_CAN_001_TX_T TxFrame;

	/* Write Frame Information */
	TxFrame.TFI = 0;
	if (pMsg->Type & CAN_REMOTE_MSG) {
		TxFrame.TFI |= CAN_TFI_RTR;
	}
	else {
		TxFrame.TFI |= CAN_TFI_DLC(pMsg->DLC);
		for (i = 0; i < (CAN_MSG_MAX_DATA_LEN + 3) / 4; i++) {
			TxFrame.TD[i] =
				pMsg->Data[4 *
						   i] |
				(pMsg->Data[4 * i +
							1] << 8) | (pMsg->Data[4 * i + 2] << 16) | (pMsg->Data[4 * i + 3] << 24);
		}
	}

	if (pMsg->ID & CAN_EXTEND_ID_USAGE) {
		TxFrame.TFI |= CAN_TFI_FF;
		TxFrame.TID = CAN_TID_ID29(pMsg->ID);
	}
	else {
		TxFrame.TID = CAN_TID_ID11(pMsg->ID);
	}

	/* Set message information */
	setSendFrameInfo(pCAN, TxBufID, &TxFrame);

	/* Select buffer and Write Transmission Request */
	if (IP_CAN_GetMode(pCAN) == CAN_MOD_STM) {
		IP_CAN_SetCmd(pCAN, CAN_CMR_STB(TxBufID) | CAN_CMR_SRR);
	}
	else {
		IP_CAN_SetCmd(pCAN, CAN_CMR_STB(TxBufID) | CAN_CMR_TR);
	}

	return SUCCESS;
}

/* Enable/Disable FullCAN interrupt */
void IP_CAN_FullCANIntConfig(IP_CAN_001_AF_T *pCanAF, FunctionalState NewState)
{
	uint32_t CurMode = IP_CAN_AF_GetMode(pCanAF);

	/* AF Off */
	IP_CAN_AF_SetMode(pCanAF, CANAF_AFMR_ACCOFF);

	if (NewState == ENABLE) {
		pCanAF->FCANIE |= CANAF_FCANIE;
	}
	else {
		pCanAF->FCANIE &= (~CANAF_FCANIE) & CANAF_FCANIE_BITMASK;
	}

	/* Return to previous mode */
	IP_CAN_AF_SetMode(pCanAF, CurMode);
}

/* Get interrupt status of the given object */
uint32_t IP_CAN_GetFullCANIntStatus(IP_CAN_001_AF_T *pCanAF, uint8_t ObjID)
{
	if (ObjID < 64) {
		return (pCanAF->FCANIC[ObjID / 32] & (1 << (ObjID % 32))) ? SET : RESET;
	}
	return RESET;
}

/* Read FullCAN message received */
Status IP_CAN_FullCANReceive(IP_CAN_001_AF_T *pCanAF, IP_CAN_001_AF_RAM_T *pCanAFRam
							 , uint8_t ObjID, IP_CAN_MSG_T *pMsg, uint8_t *pSCC) {
	uint32_t    *pSrc;
	uint16_t    FullCANEntryCnt;
	pSrc = (uint32_t *) pCanAFRam;

	FullCANEntryCnt = getTotalEntryNum(pCanAF);
	pSrc += FullCANEntryCnt + ObjID * 3;
	/* If the AF hasn't finished updating msg info */
	if (((pSrc[0] >> CANAF_FULLCAN_MSG_SEM_POS) & CANAF_FULLCAN_MSG_SEM_BITMASK) !=
		CANAF_FULCAN_MSG_AF_FINISHED) {
		return ERROR;
	}

	/* Mark that CPU is handling message */
	pSrc[0] = CANAF_FULCAN_MSG_CPU_READING << CANAF_FULLCAN_MSG_SEM_POS;

	/* Read Message */
	*pSCC = (pSrc[0] >> CANAF_FULLCAN_MSG_SCC_POS) & CANAF_FULLCAN_MSG_SCC_BITMASK;
	pMsg->ID = (pSrc[0] >> CANAF_FULLCAN_MSG_ID11_POS) & CANAF_FULLCAN_MSG_ID11_BITMASK;
	pMsg->Type = 0;
	if (pSrc[0] & (1 << CANAF_FULLCAN_MSG_RTR_POS)) {
		pMsg->Type = CAN_REMOTE_MSG;
	}
	pMsg->DLC = (pSrc[0] >> CANAF_FULLCAN_MSG_DLC_POS) & CANAF_FULLCAN_MSG_DLC_BITMASK;
	((uint32_t *) pMsg->Data)[0] = pSrc[1];
	((uint32_t *) pMsg->Data)[1] = pSrc[2];

	/* Recheck message status to make sure data is not be updated while CPU is reading */
	if (((pSrc[0] >> CANAF_FULLCAN_MSG_SEM_POS) & CANAF_FULLCAN_MSG_SEM_BITMASK) !=
		CANAF_FULCAN_MSG_CPU_READING) {
		return ERROR;
	}

	return SUCCESS;
}

/* Initialize CAN AF */
void IP_CAN_AF_Init(IP_CAN_001_AF_T *pCanAF, IP_CAN_001_AF_RAM_T *pCanAFRam) {

	IP_CAN_ClearAFLUT(pCanAF, pCanAFRam);
	IP_CAN_AF_SetMode(pCanAF, CANAF_AFMR_OPERATION);
}

/* Clear AF LUT */
void IP_CAN_ClearAFLUT(IP_CAN_001_AF_T *pCanAF, IP_CAN_001_AF_RAM_T *pCanAFRam) {
	uint32_t i = 0;
	uint32_t CurMode = IP_CAN_AF_GetMode(pCanAF);

	/*  AF Off */
	IP_CAN_AF_SetMode(pCanAF, CANAF_AFMR_ACCOFF);

	/* Clear AF Ram region */
	for (i = 0; i < CANAF_RAM_ENTRY_NUM; i++) {
		pCanAFRam->MASK[i] = 0;
	}

	/* Reset address registers */
	setSectionEndAddress(pCanAF, CANAF_RAM_FULLCAN_SEC, 0);
	setSectionEndAddress(pCanAF, CANAF_RAM_SFF_SEC, 0);
	setSectionEndAddress(pCanAF, CANAF_RAM_SFF_GRP_SEC, 0);
	setSectionEndAddress(pCanAF, CANAF_RAM_EFF_SEC, 0);
	setSectionEndAddress(pCanAF, CANAF_RAM_EFF_GRP_SEC, 0);

	/* Return to previous mode */
	IP_CAN_AF_SetMode(pCanAF, CurMode);
}

/* Set AF Lookup Table */
Status IP_CAN_SetAFLUT(IP_CAN_001_AF_T *pCanAF, IP_CAN_001_AF_RAM_T *pCanAFRam,
					   IP_CAN_AF_LUT_T *pAFSections) {
	uint16_t EntryCnt = 0, FullCANEntryCnt = 0;
	Status ret = ERROR;
	uint32_t CurMode = IP_CAN_AF_GetMode(pCanAF);

	/*  AF Off */
	IP_CAN_AF_SetMode(pCanAF, CANAF_AFMR_ACCOFF);

	do {
		/* Check a number of entries */
		if ((((pAFSections->FullCANEntryNum + 1) / 2) +
			 ((pAFSections->SffEntryNum + 1) / 2) +
			 (pAFSections->SffGrpEntryNum) +
			 (pAFSections->EffEntryNum) +
			 (pAFSections->EffGrpEntryNum * 2)) > CANAF_RAM_ENTRY_NUM) {
			ret = ERROR;
			break;
		}

		/* Setup FullCAN section */
		ret =
			setupSTDSection((uint32_t *) &pCanAFRam->MASK[EntryCnt], pAFSections->FullCANSec,
							pAFSections->FullCANEntryNum, true);
		if (ret == ERROR) {
			break;
		}
		EntryCnt = (pAFSections->FullCANEntryNum + 1) >> 1;
		FullCANEntryCnt =  EntryCnt;
		setSectionEndAddress(pCanAF, CANAF_RAM_FULLCAN_SEC, EntryCnt);

		/* Set up Individual Standard ID section */
		ret =
			setupSTDSection((uint32_t *) &pCanAFRam->MASK[EntryCnt],
							pAFSections->SffSec,
							pAFSections->SffEntryNum,
							false);
		if (ret == ERROR) {
			break;
		}

		EntryCnt += (pAFSections->SffEntryNum + 1) >> 1;
		setSectionEndAddress(pCanAF, CANAF_RAM_SFF_SEC, EntryCnt);

		/* Set up Group Standard ID section */
		ret =
			setupSTDRangeSection((uint32_t *) &pCanAFRam->MASK[EntryCnt], pAFSections->SffGrpSec,
								 pAFSections->SffGrpEntryNum);
		if (ret == ERROR) {
			break;
		}

		EntryCnt += pAFSections->SffGrpEntryNum;
		setSectionEndAddress(pCanAF, CANAF_RAM_SFF_GRP_SEC, EntryCnt);

		/* Setup Individual Extended ID section */
		ret =
			setupEXTSection((uint32_t *) &pCanAFRam->MASK[EntryCnt], pAFSections->EffSec, pAFSections->EffEntryNum);
		if (ret == ERROR) {
			break;
		}

		EntryCnt += pAFSections->EffEntryNum;
		setSectionEndAddress(pCanAF, CANAF_RAM_EFF_SEC, EntryCnt);

		/* Setup Group Extended ID section */
		ret =
			setupEXTRangeSection((uint32_t *) &pCanAFRam->MASK[EntryCnt], pAFSections->EffGrpSec,
								 pAFSections->EffGrpEntryNum);
		if (ret == ERROR) {
			break;
		}

		EntryCnt += pAFSections->EffGrpEntryNum * 2;
		setSectionEndAddress(pCanAF, CANAF_RAM_EFF_GRP_SEC, EntryCnt);

		if ((FullCANEntryCnt > 0) && ((0x800 - 6 * FullCANEntryCnt) < EntryCnt)) {
			ret = ERROR;
		}

	} while (0);
	/* Return to previous mode */
	IP_CAN_AF_SetMode(pCanAF, CurMode);

	return ret;
}

/* Get the number of entries of the given section */
uint16_t IP_CAN_GetEntriesNum(IP_CAN_001_AF_T *pCanAF, IP_CAN_001_AF_RAM_T *pCanAFRam,
							  IP_CAN_AF_RAM_SECTION_T SectionID)
{
	uint16_t StartRow, EndRow;
	uint16_t EntryCnt = 0;

	getSectionAddress(pCanAF, SectionID, &StartRow, &EndRow);
	if (EndRow <= StartRow) {
		return 0;
	}
	if ((SectionID == CANAF_RAM_FULLCAN_SEC) ||
		(SectionID == CANAF_RAM_SFF_SEC)) {
		EntryCnt = (EndRow - StartRow + 1) * 2;
		if ((((pCanAFRam->MASK[EndRow] >> CAN_STD_ENTRY_ID_POS) & CAN_STD_ENTRY_ID_MASK) ==
			 0) &&
			(((pCanAFRam->MASK[EndRow] >>
			   CAN_STD_ENTRY_DISABLE_POS) & CAN_STD_ENTRY_DISABLE_MASK) == 1)) {						/* Unsed entry */
			EntryCnt -= 1;
		}
	}
	else if ((SectionID == CANAF_RAM_SFF_GRP_SEC) ||
			 (SectionID == CANAF_RAM_EFF_SEC)) {
		EntryCnt = EndRow - StartRow + 1;
	}
	else {
		EntryCnt = (EndRow - StartRow + 1) / 2;
	}
	return EntryCnt;
}

/* Insert an entry into FullCAN Table */
Status IP_CAN_InsertSTDEntry(IP_CAN_001_AF_T *pCanAF,
							 IP_CAN_001_AF_RAM_T *pCanAFRam,
							 IP_CAN_STD_ID_Entry_T *pEntry,
							 bool IsFullCANEntry)
{
	int32_t IDIndex = 0;
	uint16_t StartRow, EndRow;
	uint16_t EntryCnt = 0;
	uint16_t i = 0;
	uint32_t tmp = 0;
	uint32_t CurMode = IP_CAN_AF_GetMode(pCanAF);

	if (getTotalEntryNum(pCanAF) >= CANAF_RAM_ENTRY_NUM) {
		return ERROR;
	}

	/* Check if a number of entries in  section is odd or even */
	if (IsFullCANEntry) {
		getSectionAddress(pCanAF, CANAF_RAM_FULLCAN_SEC, &StartRow, &EndRow);
	}
	else {
		getSectionAddress(pCanAF, CANAF_RAM_SFF_SEC, &StartRow, &EndRow);

	}

	if (EndRow > StartRow) {
		EntryCnt = (EndRow - StartRow + 1) * 2;
		if ((((pCanAFRam->MASK[EndRow] >> CAN_STD_ENTRY_ID_POS) & CAN_STD_ENTRY_ID_MASK) ==
			 0) &&
			(((pCanAFRam->MASK[EndRow] >>
			   CAN_STD_ENTRY_DISABLE_POS) & CAN_STD_ENTRY_DISABLE_MASK) == 1)) {						/* Unsed entry */
			EntryCnt -= 1;
		}

	}

	/* Search for Index of new entry */
	IDIndex = searchInsertIndex((uint32_t *) &pCanAFRam->MASK[StartRow],
								EntryCnt,
								pEntry->ID_11 & CAN_STD_ENTRY_ID_MASK,
								CAN_STD_ENTRY_ID_MASK,
								sizeof(uint16_t));
	if ((IDIndex == -1) || (IDIndex > EntryCnt )) {
		return ERROR;
	}

	/*  AF Off */
	IP_CAN_AF_SetMode(pCanAF, CANAF_AFMR_ACCOFF);

	/* Move all remaining sections one place down
	            if new entry will increase FullCAN list */
	if ((EntryCnt % 2) == 0) {
		uint16_t StartAddr, EndAddr;

		for (i = getTotalEntryNum(pCanAF); i > EndRow; i--) {
			pCanAFRam->MASK[i] = pCanAFRam->MASK[i - 1];
		}

		if (IsFullCANEntry) {
			getSectionAddress(pCanAF, CANAF_RAM_FULLCAN_SEC, &StartAddr, &EndAddr);
			setSectionEndAddress(pCanAF, CANAF_RAM_FULLCAN_SEC, EndAddr + 2);
		}
		getSectionAddress(pCanAF, CANAF_RAM_SFF_SEC, &StartAddr, &EndAddr);
		setSectionEndAddress(pCanAF, CANAF_RAM_SFF_SEC, EndAddr + 2);
		getSectionAddress(pCanAF, CANAF_RAM_SFF_GRP_SEC, &StartAddr, &EndAddr);
		setSectionEndAddress(pCanAF, CANAF_RAM_SFF_GRP_SEC, EndAddr + 2);
		getSectionAddress(pCanAF, CANAF_RAM_EFF_SEC, &StartAddr, &EndAddr);
		setSectionEndAddress(pCanAF, CANAF_RAM_EFF_SEC, EndAddr + 2);
		getSectionAddress(pCanAF, CANAF_RAM_EFF_GRP_SEC, &StartAddr, &EndAddr);
		setSectionEndAddress(pCanAF, CANAF_RAM_EFF_GRP_SEC, EndAddr + 2);
	}

	/* Shift rows behind the row of search index. If search index  is low index of the row, shift the row of search index also. */
	if ((IDIndex % 2) == 0) {
		shiftSTDEntryDown((uint32_t *) &pCanAFRam->MASK[StartRow + IDIndex / 2], EntryCnt - IDIndex);
	}
	else {
		shiftSTDEntryDown((uint32_t *) &pCanAFRam->MASK[StartRow + IDIndex / 2 + 1], EntryCnt - IDIndex - 1);
	}

	/* Insert new item */
	tmp = createStdIDEntry(pEntry, IsFullCANEntry);
	if ((IDIndex % 2) == 0) {
		if (IDIndex == EntryCnt) {
			/* Insert unused item if the new item is the last item*/
			pCanAFRam->MASK[StartRow + IDIndex / 2] = (tmp << 16) | createUnUsedSTDEntry(pEntry->CtrlNo);
		}
		else {
			uint32_t val;
			val = pCanAFRam->MASK[StartRow + IDIndex / 2] & 0x0000FFFF;
			/* Insert new item */
			pCanAFRam->MASK[StartRow + IDIndex / 2] = val | (tmp << 16);
		}
	}
	else {
		uint32_t val, valNext;
		val = pCanAFRam->MASK[StartRow + IDIndex / 2];
		valNext = pCanAFRam->MASK[StartRow + IDIndex / 2 + 1];
		/* In case the new entry is not the last item, shift the item at the found index to the next row*/
		if (IDIndex < EntryCnt ) {

			pCanAFRam->MASK[StartRow + IDIndex / 2 + 1] = (valNext & 0x0000FFFF) | ((val & 0xFFFF) << 16);
		}
		pCanAFRam->MASK[StartRow + IDIndex / 2] = (val & 0xFFFF0000) | tmp;
	}

	/* Return to previous mode */
	IP_CAN_AF_SetMode(pCanAF, CurMode);
	return SUCCESS;
}

/* Insert an entry into FullCAN Table */
Status IP_CAN_InsertFullCANEntry(IP_CAN_001_AF_T *pCanAF,
								 IP_CAN_001_AF_RAM_T *pCanAFRam,
								 IP_CAN_STD_ID_Entry_T *pEntry) {
	return IP_CAN_InsertSTDEntry(pCanAF, pCanAFRam, pEntry, true);
}

/* Insert an entry into Individual STD section */
Status IP_CAN_InsertIndividualSTDEntry(IP_CAN_001_AF_T *pCanAF,
									   IP_CAN_001_AF_RAM_T *pCanAFRam,
									   IP_CAN_STD_ID_Entry_T *pEntry)
{
	return IP_CAN_InsertSTDEntry(pCanAF, pCanAFRam, pEntry, false);
}

/* Insert an entry into Individual EXT section */
Status IP_CAN_InsertIndividualEXTEntry(IP_CAN_001_AF_T *pCanAF,
									   IP_CAN_001_AF_RAM_T *pCanAFRam,
									   IP_CAN_EXT_ID_Entry_T *pEntry) {
	int32_t IDIndex = 0;
	uint16_t StartRow, EndRow;
	uint16_t EntryCnt = 0;
	uint16_t i = 0;
	uint32_t tmp = 0;
	uint32_t CurMode = IP_CAN_AF_GetMode(pCanAF);

	if (getTotalEntryNum(pCanAF) >= CANAF_RAM_ENTRY_NUM) {
		return ERROR;
	}

	/* Check if a number of entries in  section is odd or even */
	getSectionAddress(pCanAF, CANAF_RAM_EFF_SEC, &StartRow, &EndRow);

	if (EndRow > StartRow) {
		EntryCnt = EndRow - StartRow + 1;
	}

	/* Search for Index of new entry */
	IDIndex = searchInsertIndex((uint32_t *) &pCanAFRam->MASK[StartRow],
								EntryCnt,
								pEntry->ID_29 & CAN_EXT_ENTRY_ID_MASK,
								CAN_EXT_ENTRY_ID_MASK,
								sizeof(uint32_t));
	if ((IDIndex == -1) || (IDIndex > EntryCnt )) {
		return ERROR;
	}

	/*  AF Off */
	IP_CAN_AF_SetMode(pCanAF, CANAF_AFMR_ACCOFF);

	/* Move all remaining sections one place down
	            if new entry will increase FullCAN list */

	for (i = getTotalEntryNum(pCanAF); i > (StartRow + IDIndex); i--) {
		pCanAFRam->MASK[i] = pCanAFRam->MASK[i - 1];
	}

	/* Insert new item */
	tmp = createExtIDEntry(pEntry);
	pCanAFRam->MASK[StartRow + IDIndex] = tmp;

	/* Update address table */
	getSectionAddress(pCanAF, CANAF_RAM_EFF_SEC, &StartRow, &EndRow);
	setSectionEndAddress(pCanAF, CANAF_RAM_EFF_SEC, EndRow + 2);
	getSectionAddress(pCanAF, CANAF_RAM_EFF_GRP_SEC, &StartRow, &EndRow);
	setSectionEndAddress(pCanAF, CANAF_RAM_EFF_GRP_SEC, EndRow + 2);

	/* Return to previous mode */
	IP_CAN_AF_SetMode(pCanAF, CurMode);
	return SUCCESS;
}

Status IP_CAN_InsertGroupSTDEntry(IP_CAN_001_AF_T *pCanAF,
								  IP_CAN_001_AF_RAM_T *pCanAFRam,
								  IP_CAN_STD_ID_RANGE_Entry_T *pEntry)
{
	uint16_t InsertIndex = 0;
	uint16_t StartRow, EndRow;
	uint16_t LowerID = 0, UpperID = 0;
	uint16_t i = 0;
	uint32_t CurMode = IP_CAN_AF_GetMode(pCanAF);

	if (getTotalEntryNum(pCanAF) >= CANAF_RAM_ENTRY_NUM) {
		return ERROR;
	}

	getSectionAddress(pCanAF, CANAF_RAM_SFF_GRP_SEC, &StartRow, &EndRow);

	/* Search for Index of the entry which upper the new item */
	for (InsertIndex = StartRow; InsertIndex <= EndRow; InsertIndex++) {
		LowerID = (pCanAFRam->MASK[InsertIndex] >> (16 + CAN_STD_ENTRY_ID_POS)) & CAN_STD_ENTRY_ID_MASK;
		UpperID = (pCanAFRam->MASK[InsertIndex] >> CAN_STD_ENTRY_ID_POS) & CAN_STD_ENTRY_ID_MASK;
		if (LowerID >= pEntry->LowerID.ID_11) {
			break;
		}
	}

	/* Compare to the previous row (if any)*/
	if (InsertIndex > 0) {
		uint16_t PrevUpperID;
		PrevUpperID = (pCanAFRam->MASK[InsertIndex - 1] >> CAN_STD_ENTRY_ID_POS) & CAN_STD_ENTRY_ID_MASK;

		if (PrevUpperID >= pEntry->UpperID.ID_11) {
			return SUCCESS;
		}

		if (pEntry->UpperID.ID_11 < LowerID) {
			if (pEntry->LowerID.ID_11 < PrevUpperID) {	/* The new range is merged to the range of the previous row */
				uint32_t val = pCanAFRam->MASK[InsertIndex - 1] & 0xFFFF0000;
				pCanAFRam->MASK[InsertIndex - 1] = val | createStdIDEntry(&pEntry->UpperID, false);
				return SUCCESS;
			}
			else {
				goto insert_grp_entry;
			}
		}
	}

	/* Compare to the next row  (if any)*/
	if ((EndRow) && (InsertIndex <= EndRow)) {
		if (pEntry->UpperID.ID_11 >= UpperID) {	/* The new range is merged to the range of the next row */
			uint32_t val;
			val = createStdIDEntry(&pEntry->LowerID, false) << 16;
			val |= createStdIDEntry(&pEntry->UpperID, false);
			pCanAFRam->MASK[InsertIndex] = val;
			return SUCCESS;
		}
		else if (pEntry->UpperID.ID_11 < UpperID) {
			if (pEntry->UpperID.ID_11 > LowerID) {	/* The new range is merged to the range of the next row */
				uint32_t val = pCanAFRam->MASK[InsertIndex] & 0x0000FFFF;
				pCanAFRam->MASK[InsertIndex] = val | (createStdIDEntry(&pEntry->LowerID, false) << 16);
				return SUCCESS;
			}
		}
	}

insert_grp_entry:

	/*  AF Off */
	IP_CAN_AF_SetMode(pCanAF, CANAF_AFMR_ACCOFF);

	/* Move all remaining sections one place down
	            if new entry will increase FullCAN list */

	for (i = getTotalEntryNum(pCanAF); i > InsertIndex; i--) {
		pCanAFRam->MASK[i] = pCanAFRam->MASK[i - 1];
	}

	/* Insert new item */
	pCanAFRam->MASK[InsertIndex] = createStdIDEntry(&pEntry->LowerID, false) << 16;
	pCanAFRam->MASK[InsertIndex] |= createStdIDEntry(&pEntry->UpperID, false);

	getSectionAddress(pCanAF, CANAF_RAM_SFF_GRP_SEC, &StartRow, &EndRow);
	setSectionEndAddress(pCanAF, CANAF_RAM_SFF_GRP_SEC, EndRow + 2);
	getSectionAddress(pCanAF, CANAF_RAM_EFF_SEC, &StartRow, &EndRow);
	setSectionEndAddress(pCanAF, CANAF_RAM_EFF_SEC, EndRow + 2);
	getSectionAddress(pCanAF, CANAF_RAM_EFF_GRP_SEC, &StartRow, &EndRow);
	setSectionEndAddress(pCanAF, CANAF_RAM_EFF_GRP_SEC, EndRow + 2);

	/* Return to previous mode */
	IP_CAN_AF_SetMode(pCanAF, CurMode);
	return SUCCESS;
}

Status IP_CAN_InsertGroupEXTEntry(IP_CAN_001_AF_T *pCanAF,
								  IP_CAN_001_AF_RAM_T *pCanAFRam,
								  IP_CAN_EXT_ID_RANGE_Entry_T *pEntry)
{
	uint32_t InsertIndex = 0;
	uint16_t StartRow, EndRow;
	uint32_t LowerID = 0, UpperID = 0;
	uint16_t i = 0;
	uint32_t CurMode = IP_CAN_AF_GetMode(pCanAF);

	if (getTotalEntryNum(pCanAF) >= CANAF_RAM_ENTRY_NUM) {
		return ERROR;
	}

	/* Check if a number of entries in  section is odd or even */
	getSectionAddress(pCanAF, CANAF_RAM_EFF_GRP_SEC, &StartRow, &EndRow);

	/* Search for Index of new entry */
	for (InsertIndex = StartRow; InsertIndex <= EndRow; InsertIndex += 2) {
		LowerID = (pCanAFRam->MASK[InsertIndex] >> CAN_EXT_ENTRY_ID_POS) & CAN_EXT_ENTRY_ID_MASK;
		UpperID = (pCanAFRam->MASK[InsertIndex + 1] >> CAN_EXT_ENTRY_ID_POS) & CAN_EXT_ENTRY_ID_MASK;
		if (LowerID >= pEntry->LowerID.ID_29) {
			break;
		}
	}

	/* Compare to the previous row (if any)*/
	if (InsertIndex > 0) {
		uint32_t PrevUpperID;
		PrevUpperID = (pCanAFRam->MASK[(InsertIndex - 2) + 1] >> CAN_EXT_ENTRY_ID_POS) & CAN_EXT_ENTRY_ID_MASK;

		if (PrevUpperID >= pEntry->UpperID.ID_29) {
			return SUCCESS;
		}

		if (pEntry->UpperID.ID_29 < LowerID) {
			if (pEntry->LowerID.ID_29 < PrevUpperID) {
				pCanAFRam->MASK[(InsertIndex - 2) + 1] = createExtIDEntry(&pEntry->UpperID);
				return SUCCESS;
			}
			else {
				goto insert_grp_entry;
			}
		}
	}

	if ((EndRow) && (InsertIndex < EndRow)) {
		if (pEntry->UpperID.ID_29 >= UpperID) {
			pCanAFRam->MASK[InsertIndex] = createExtIDEntry(&pEntry->LowerID);
			pCanAFRam->MASK[InsertIndex + 1] = createExtIDEntry(&pEntry->UpperID);
			return SUCCESS;
		}
		else if (pEntry->UpperID.ID_29 < UpperID) {
			if (pEntry->UpperID.ID_29 > LowerID) {
				pCanAFRam->MASK[InsertIndex] = createExtIDEntry(&pEntry->LowerID);
				return SUCCESS;
			}
		}
	}

insert_grp_entry:

	/*  AF Off */
	IP_CAN_AF_SetMode(pCanAF, CANAF_AFMR_ACCOFF);

	/* Move all remaining sections two places down
	            if new entry will increase FullCAN list */

	for (i = getTotalEntryNum(pCanAF) + 1; i > InsertIndex; i--) {
		pCanAFRam->MASK[i] = pCanAFRam->MASK[i - 2];
	}

	/* Insert new item */
	pCanAFRam->MASK[InsertIndex] = createExtIDEntry(&pEntry->LowerID);
	pCanAFRam->MASK[InsertIndex + 1] = createExtIDEntry(&pEntry->UpperID);

	getSectionAddress(pCanAF, CANAF_RAM_EFF_GRP_SEC, &StartRow, &EndRow);
	setSectionEndAddress(pCanAF, CANAF_RAM_EFF_GRP_SEC, EndRow + 3);

	/* Return to previous mode */
	IP_CAN_AF_SetMode(pCanAF, CurMode);
	return SUCCESS;
}

/* Remove STD Entry from given table */
Status IP_CAN_RemoveSTDEntry(IP_CAN_001_AF_T *pCanAF,
							 IP_CAN_001_AF_RAM_T *pCanAFRam,
							 int16_t IDIndex,
							 bool IsFullCANEntry)
{
	uint16_t StartRow, EndRow;
	uint16_t EntryCnt = 0, i;
	uint32_t tmp = 0;
	uint32_t CurMode = IP_CAN_AF_GetMode(pCanAF);

	if (IDIndex < 0) {
		return ERROR;
	}

	if (getTotalEntryNum(pCanAF) >= CANAF_RAM_ENTRY_NUM) {
		return ERROR;
	}

	/* Check if a number of entries in  section is odd or even */
	if (IsFullCANEntry) {
		getSectionAddress(pCanAF, CANAF_RAM_FULLCAN_SEC, &StartRow, &EndRow);
	}
	else {
		getSectionAddress(pCanAF, CANAF_RAM_SFF_SEC, &StartRow, &EndRow);
	}

	if (EndRow > StartRow) {
		EntryCnt = (EndRow - StartRow + 1) * 2;
		if ((((pCanAFRam->MASK[EndRow] >> CAN_STD_ENTRY_ID_POS) & CAN_STD_ENTRY_ID_MASK) ==
			 0) &&
			(((pCanAFRam->MASK[EndRow] >>
			   CAN_STD_ENTRY_DISABLE_POS) & CAN_STD_ENTRY_DISABLE_MASK) == 1)) {						/* Unsed entry */
			EntryCnt -= 1;
		}

	}
	if (IDIndex >= EntryCnt) {
		return ERROR;
	}

	/*  AF Off */
	IP_CAN_AF_SetMode(pCanAF, CANAF_AFMR_ACCOFF);

	/* Shift rows behind the row of remove index. If remove index  is low index of the row, shift the row of remove index also. */
	tmp = pCanAFRam->MASK[StartRow + IDIndex / 2 + 1];
	if ((IDIndex % 2) == 0) {
		shiftSTDEntryUp((uint32_t *) &pCanAFRam->MASK[StartRow + IDIndex / 2], EntryCnt - IDIndex);
	}
	else {
		shiftSTDEntryUp((uint32_t *) &pCanAFRam->MASK[StartRow + IDIndex / 2 + 1], EntryCnt - IDIndex - 1);

		/* Set value for the entry at remove index */
		pCanAFRam->MASK[StartRow + IDIndex / 2] &= 0xFFFF0000;
		if (IDIndex == (EntryCnt - 1)) {
			uint8_t CtrlNo;
			tmp = (pCanAFRam->MASK[StartRow + IDIndex / 2]) >> 16;
			CtrlNo = (tmp >> CAN_STD_ENTRY_CTRL_NO_POS) & CAN_STD_ENTRY_CTRL_NO_MASK;
			pCanAFRam->MASK[StartRow + IDIndex / 2] |= createUnUsedSTDEntry(CtrlNo);
		}
		else {
			pCanAFRam->MASK[StartRow + IDIndex / 2] |= (tmp >> 16) & 0xFFFF;
		}
	}
	/* Move all remaining sections one place up
	            if new entry will decrease FullCAN list */
	if (EntryCnt % 2) {
		uint16_t StartAddr, EndAddr;

		for (i = EndRow; i < getTotalEntryNum(pCanAF); i++) {
			pCanAFRam->MASK[i] = pCanAFRam->MASK[i + 1];
		}

		if (IsFullCANEntry) {
			getSectionAddress(pCanAF, CANAF_RAM_FULLCAN_SEC, &StartAddr, &EndAddr);
			setSectionEndAddress(pCanAF, CANAF_RAM_FULLCAN_SEC, EndAddr);
		}
		getSectionAddress(pCanAF, CANAF_RAM_SFF_SEC, &StartAddr, &EndAddr);
		setSectionEndAddress(pCanAF, CANAF_RAM_SFF_SEC, EndAddr);
		getSectionAddress(pCanAF, CANAF_RAM_SFF_GRP_SEC, &StartAddr, &EndAddr);
		setSectionEndAddress(pCanAF, CANAF_RAM_SFF_GRP_SEC, EndAddr);
		getSectionAddress(pCanAF, CANAF_RAM_EFF_SEC, &StartAddr, &EndAddr);
		setSectionEndAddress(pCanAF, CANAF_RAM_EFF_SEC, EndAddr);
		getSectionAddress(pCanAF, CANAF_RAM_EFF_GRP_SEC, &StartAddr, &EndAddr);
		setSectionEndAddress(pCanAF, CANAF_RAM_EFF_GRP_SEC, EndAddr);
	}

	/* Return to previous mode */
	IP_CAN_AF_SetMode(pCanAF, CurMode);
	return SUCCESS;
}

/* Remove LUT Entry from given table */
Status IP_CAN_RemoveLUTEntry(IP_CAN_001_AF_T *pCanAF,
							 IP_CAN_001_AF_RAM_T *pCanAFRam,
							 IP_CAN_AF_RAM_SECTION_T SectionID,
							 int16_t Position)
{
	uint16_t StartRow, EndRow;
	uint16_t StartAddr, EndAddr;
	uint16_t EntryCnt = 0;
	uint8_t  EntryRowNum = 1;
	uint16_t i = 0;
	uint32_t CurMode = IP_CAN_AF_GetMode(pCanAF);

	if (Position < 0) {
		return ERROR;
	}

	if (getTotalEntryNum(pCanAF) >= CANAF_RAM_ENTRY_NUM) {
		return ERROR;
	}

	if (SectionID == CANAF_RAM_FULLCAN_SEC) {
		return IP_CAN_RemoveSTDEntry(pCanAF, pCanAFRam, Position, true);
	}
	else if (SectionID == CANAF_RAM_SFF_SEC) {
		return IP_CAN_RemoveSTDEntry(pCanAF, pCanAFRam, Position, false);
	}

	/* Get a number of rows for an entry */
	if (SectionID == CANAF_RAM_EFF_GRP_SEC) {
		EntryRowNum = 2;
	}

	/* Get Start Row, End Row */
	getSectionAddress(pCanAF, SectionID, &StartRow, &EndRow);

	if (EndRow > StartRow) {
		EntryCnt = (EndRow - StartRow + 1) / EntryRowNum;
	}

	if (Position >= EntryCnt) {
		return ERROR;
	}

	/*  AF Off */
	IP_CAN_AF_SetMode(pCanAF, CANAF_AFMR_ACCOFF);

	/* Move all remaining sections one place up
	            if new entry will increase FullCAN list */
	for (i = StartRow + Position * EntryRowNum; i < getTotalEntryNum(pCanAF); i++) {
		pCanAFRam->MASK[i] = pCanAFRam->MASK[i + EntryRowNum];
	}

	/* Get Start Row, End Row */
	switch (SectionID) {
	case CANAF_RAM_SFF_GRP_SEC:
		getSectionAddress(pCanAF, CANAF_RAM_SFF_GRP_SEC, &StartAddr, &EndAddr);
		setSectionEndAddress(pCanAF, CANAF_RAM_SFF_GRP_SEC, EndAddr - EntryRowNum + 1);

	case CANAF_RAM_EFF_SEC:
		getSectionAddress(pCanAF, CANAF_RAM_EFF_SEC, &StartAddr, &EndAddr);
		setSectionEndAddress(pCanAF, CANAF_RAM_EFF_SEC, EndAddr - EntryRowNum + 1);

	case CANAF_RAM_EFF_GRP_SEC:
		getSectionAddress(pCanAF, CANAF_RAM_EFF_GRP_SEC, &StartAddr, &EndAddr);
		setSectionEndAddress(pCanAF, CANAF_RAM_EFF_GRP_SEC, EndAddr - EntryRowNum + 1);
		break;
	default:
		return ERROR;
	}

	/* Return to previous mode */
	IP_CAN_AF_SetMode(pCanAF, CurMode);
	return SUCCESS;
}

/* Remove an entry into FullCAN Table */
Status IP_CAN_RemoveFullCANEntry(IP_CAN_001_AF_T *pCanAF,
								 IP_CAN_001_AF_RAM_T *pCanAFRam,
								 int16_t Position) {
	return IP_CAN_RemoveLUTEntry(pCanAF, pCanAFRam, CANAF_RAM_FULLCAN_SEC, Position);
}

/* Remove an entry into Individual STD section */
Status IP_CAN_RemoveIndividualSTDEntry(IP_CAN_001_AF_T *pCanAF,
									   IP_CAN_001_AF_RAM_T *pCanAFRam,
									   int16_t Position)
{
	return IP_CAN_RemoveLUTEntry(pCanAF, pCanAFRam, CANAF_RAM_SFF_SEC, Position);
}

/* Remove an entry into Group STD section */
Status IP_CAN_RemoveGroupSTDEntry(IP_CAN_001_AF_T *pCanAF,
								  IP_CAN_001_AF_RAM_T *pCanAFRam,
								  int16_t Position)
{
	return IP_CAN_RemoveLUTEntry(pCanAF, pCanAFRam, CANAF_RAM_SFF_GRP_SEC, Position);
}

/* Remove an entry into Individual EXT section */
Status IP_CAN_RemoveIndividualEXTEntry(IP_CAN_001_AF_T *pCanAF,
									   IP_CAN_001_AF_RAM_T *pCanAFRam,
									   int16_t Position) {
	return IP_CAN_RemoveLUTEntry(pCanAF, pCanAFRam, CANAF_RAM_EFF_SEC, Position);
}

/* Remove an entry into Group EXT section */
Status IP_CAN_RemoveGroupEXTEntry(IP_CAN_001_AF_T *pCanAF,
								  IP_CAN_001_AF_RAM_T *pCanAFRam,
								  int16_t Position)
{
	return IP_CAN_RemoveLUTEntry(pCanAF, pCanAFRam, CANAF_RAM_EFF_GRP_SEC, Position);
}

/* Read STD Entry */
Status IP_CAN_ReadSTDEntry(IP_CAN_001_AF_T *pCanAF,
						   IP_CAN_001_AF_RAM_T *pCanAFRam,
						   uint16_t Position,
						   bool   IsFullCANEntry,
						   IP_CAN_STD_ID_Entry_T *pEntry)

{
	uint16_t StartRow, EndRow;
	uint16_t EntryCnt = 0;

	/* Check if a number of entries in  section is odd or even */
	if (IsFullCANEntry) {
		getSectionAddress(pCanAF, CANAF_RAM_FULLCAN_SEC, &StartRow, &EndRow);
	}
	else {
		getSectionAddress(pCanAF, CANAF_RAM_SFF_SEC, &StartRow, &EndRow);
	}

	if (EndRow > StartRow) {
		EntryCnt = (EndRow - StartRow + 1) * 2;
		if ((((pCanAFRam->MASK[EndRow] >> CAN_STD_ENTRY_ID_POS) & CAN_STD_ENTRY_ID_MASK) ==
			 0) &&
			(((pCanAFRam->MASK[EndRow] >>
			   CAN_STD_ENTRY_DISABLE_POS) & CAN_STD_ENTRY_DISABLE_MASK) == 1)) {						/* Unsed entry */
			EntryCnt -= 1;
		}
	}
	if (Position >= EntryCnt) {
		return ERROR;
	}

	if ((Position % 2) == 0) {
		readStdIDEntry(pCanAFRam->MASK[StartRow + Position / 2] >> 16, pEntry);
	}
	else {
		readStdIDEntry(pCanAFRam->MASK[StartRow + Position / 2] & 0xFFFF, pEntry);
	}

	return SUCCESS;
}

Status IP_CAN_ReadFullCANEntry(IP_CAN_001_AF_T *pCanAF,
							   IP_CAN_001_AF_RAM_T *pCanAFRam,
							   uint16_t Position,
							   IP_CAN_STD_ID_Entry_T *pEntry)
{
	return IP_CAN_ReadSTDEntry(pCanAF, pCanAFRam, Position, true, pEntry);
}

Status IP_CAN_ReadIndividualSTDEntry(IP_CAN_001_AF_T *pCanAF,
									 IP_CAN_001_AF_RAM_T *pCanAFRam,
									 uint16_t Position,
									 IP_CAN_STD_ID_Entry_T *pEntry)
{
	return IP_CAN_ReadSTDEntry(pCanAF, pCanAFRam, Position, false, pEntry);
}

Status IP_CAN_ReadGroupSTDEntry(IP_CAN_001_AF_T *pCanAF,
								IP_CAN_001_AF_RAM_T *pCanAFRam,
								uint16_t Position,
								IP_CAN_STD_ID_RANGE_Entry_T *pEntry)
{
	uint16_t StartRow, EndRow;
	uint16_t EntryCnt = 0;

	getSectionAddress(pCanAF, CANAF_RAM_SFF_GRP_SEC, &StartRow, &EndRow);

	if (EndRow > StartRow) {
		EntryCnt = EndRow - StartRow + 1;
	}
	if (Position >= EntryCnt) {
		return ERROR;
	}

	readStdIDEntry(pCanAFRam->MASK[StartRow + Position] >> 16, &pEntry->LowerID);
	readStdIDEntry(pCanAFRam->MASK[StartRow + Position] & 0xFFFF, &pEntry->UpperID);
	return SUCCESS;
}

Status IP_CAN_ReadIndividualEXTEntry(IP_CAN_001_AF_T *pCanAF,
									 IP_CAN_001_AF_RAM_T *pCanAFRam,
									 uint16_t Position,
									 IP_CAN_EXT_ID_Entry_T *pEntry)
{
	uint16_t StartRow, EndRow;
	uint16_t EntryCnt = 0;

	getSectionAddress(pCanAF, CANAF_RAM_EFF_SEC, &StartRow, &EndRow);

	if (EndRow > StartRow) {
		EntryCnt = EndRow - StartRow + 1;
	}
	if (Position >= EntryCnt) {
		return ERROR;
	}

	readExtIDEntry(pCanAFRam->MASK[StartRow + Position], pEntry);
	return SUCCESS;
}

Status IP_CAN_ReadGroupEXTEntry(IP_CAN_001_AF_T *pCanAF,
								IP_CAN_001_AF_RAM_T *pCanAFRam,
								uint16_t Position,
								IP_CAN_EXT_ID_RANGE_Entry_T *pEntry)

{
	uint16_t StartRow, EndRow;
	uint16_t EntryCnt = 0;

	getSectionAddress(pCanAF, CANAF_RAM_EFF_GRP_SEC, &StartRow, &EndRow);

	if (EndRow > StartRow) {
		EntryCnt = (EndRow - StartRow + 1) / 2;
	}
	if (Position >= EntryCnt) {
		return ERROR;
	}

	readExtIDEntry(pCanAFRam->MASK[StartRow + Position * 2], &pEntry->LowerID);
	readExtIDEntry(pCanAFRam->MASK[StartRow + Position * 2 + 1], &pEntry->UpperID);
	return SUCCESS;
}
