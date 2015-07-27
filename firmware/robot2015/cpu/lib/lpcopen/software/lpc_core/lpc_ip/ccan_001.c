/*
 * @brief CCAN Registers and control functions
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

#include "ccan_001.h"

/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/

/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/

/*****************************************************************************
 * Private functions
 ****************************************************************************/
/* Macro for reading specific RAM message object to IF */
static void CCAN_IF_Buf_transfer(IP_CCAN_001_T *pCCAN,
								 IP_CCAN_MSG_INTERFACE_T IFsel,
								 uint32_t msg_num,
								 uint8_t direction) {
	msg_num &= 0x3F;
	CCAN_IF_Write(pCCAN, CMDMSK_W, IFsel,
				  (CCAN_RW(direction) | CCAN_MASK | CCAN_ARB | CCAN_CTRL | CCAN_CLRINTPND | CCAN_DATAA | CCAN_DATAB));
	CCAN_IF_Write(pCCAN, CMDREQ, IFsel, msg_num);
	while ((CCAN_IF_Read(pCCAN, CMDREQ, IFsel)) & CCAN_IFCREQ_BUSY ) {}
}

/*****************************************************************************
 * Public functions
 ****************************************************************************/

/* Initialize the CAN controller */
void IP_CCAN_SWInit(IP_CCAN_001_T *pCCAN, FunctionalState NewState)
{
	if (NewState == ENABLE) {
		pCCAN->CNTL |= CCAN_CTRL_INIT;
	}
	else {
		pCCAN->CNTL &= ~CCAN_CTRL_INIT;		/* Initialization finished, normal operation now. */
		while ( pCCAN->CNTL & CCAN_CTRL_INIT ) {}
	}
}

/* Configure the bit timing for CCAN bus */
void IP_CCAN_TimingCfg(IP_CCAN_001_T *pCCAN,
					   uint32_t ClkDiv,
					   uint32_t BaudRatePrescaler,
					   uint8_t SynJumpWidth,
					   uint8_t Tseg1,
					   uint8_t Tseg2)
{
	IP_CCAN_SWInit(pCCAN, ENABLE);
	pCCAN->CLKDIV = ClkDiv;			/* Divider for CAN VPB3 clock */
	pCCAN->CNTL |= CCAN_CTRL_CCE;		/* Start configuring bit timing */
	pCCAN->BT = (BaudRatePrescaler & 0x3F) | (SynJumpWidth & 0x03) << 6 | (Tseg1 & 0x0F) << 8 | (Tseg2 & 0x07) << 12;
	pCCAN->BRPE = BaudRatePrescaler >> 6;	/* Set Baud Rate Prescaler MSBs */
	pCCAN->CNTL &= ~CCAN_CTRL_CCE;		/* Stop configuring bit timing */
	IP_CCAN_SWInit(pCCAN, DISABLE);
}

/* Enable/Disable CCAN Interrupts */
void IP_CCAN_IntEnable(IP_CCAN_001_T *pCCAN, IP_CCAN_INT_T Int_type, FunctionalState NewState)
{
	if (NewState == ENABLE) {
		pCCAN->CNTL |= Int_type;
	}
	else {
		pCCAN->CNTL &= ~Int_type;
	}
}

/* Enable/Disable automatic retransmission */
void IP_CCAN_AutoRetransmitEnable(IP_CCAN_001_T *pCCAN, FunctionalState NewState)
{
	if (NewState == ENABLE) {
		pCCAN->CNTL &= ~CCAN_CTRL_DAR;
	}
	else {
		pCCAN->CNTL |= CCAN_CTRL_DAR;
	}
}

/* Get the current value of the transmit/receive error counter */
uint8_t IP_CCAN_GetErrCounter(IP_CCAN_001_T *pCCAN, IP_CCAN_TRX_MODE_T TRMode)
{
	return (TRMode == CCAN_TX_MODE) ? (pCCAN->EC & 0x0FF) : ((pCCAN->EC >> 8) & 0x0FF);	// TODO: Confirm bit number of TEC_7_0 and REC_6_0
}

/* Get the CCAN status register */
uint32_t IP_CCAN_GetStatus(IP_CCAN_001_T *pCCAN)
{
	return pCCAN->STAT;
}

/* Set the CCAN status */
void IP_CCAN_SetStatus(IP_CCAN_001_T *pCCAN, uint32_t val)
{
	pCCAN->STAT = val & 0x1F;
}

/* Get the source ID of an interrupt */
uint32_t IP_CCAN_Get_IntID(IP_CCAN_001_T *pCCAN)
{
	return pCCAN->INT;
}

/* Enable/Disable test mode in CCAN */
void IP_CCAN_TestModeEnable(IP_CCAN_001_T *pCCAN, IP_CCAN_TEST_MODE_T test_mode, FunctionalState NewState)
{
	if (NewState == ENABLE) {
		pCCAN->CNTL |= CCAN_CTRL_TEST;
		pCCAN->TEST |= test_mode;
	}
	else {
		pCCAN->CNTL &= ~CCAN_CTRL_TEST;
		pCCAN->TEST &= ~test_mode;
	}
}

/* Clear interrupt pending bit in the message object */
void IP_CCAN_ClearIntPend(IP_CCAN_001_T *pCCAN, IP_CCAN_MSG_INTERFACE_T IFsel, uint8_t msg_num)
{
	msg_num &= 0x3F;
	CCAN_IF_Write(pCCAN, CMDMSK_R, IFsel, CCAN_RD | CCAN_CLRINTPND);
	CCAN_IF_Write(pCCAN, CMDREQ, IFsel, msg_num);
	while (CCAN_IF_Read(pCCAN, CMDREQ, IFsel) & CCAN_IFCREQ_BUSY ) {}
}

/* Clear new data flag bit in the message object */
void IP_CCAN_Clear_NewDataFlag(IP_CCAN_001_T *pCCAN, IP_CCAN_MSG_INTERFACE_T IFsel, uint8_t msg_num)
{
	msg_num &= 0x3F;
	CCAN_IF_Write(pCCAN, CMDMSK_R, IFsel, CCAN_RD | CCAN_NEWDAT);
	CCAN_IF_Write(pCCAN, CMDREQ, IFsel, msg_num);
	while (CCAN_IF_Read(pCCAN, CMDREQ, IFsel) & CCAN_IFCREQ_BUSY ) {}
}

/* Enable/Disable the message object to valid */
void IP_CCAN_SetValidMsg(IP_CCAN_001_T *pCCAN, IP_CCAN_MSG_INTERFACE_T IFsel, uint8_t msg_num, FunctionalState NewState)
{

	uint32_t temp;
	temp = CCAN_IF_Read(pCCAN, ARB2, IFsel);
	if (NewState == DISABLE) {
		CCAN_IF_Write(pCCAN, ARB2, IFsel, (temp & (~CCAN_ID_MVAL)));
	}
	else {
		CCAN_IF_Write(pCCAN, ARB2, IFsel, (temp | (CCAN_ID_MVAL)));
	}
	CCAN_IF_Write(pCCAN, CMDMSK_W, IFsel, CCAN_RW(CCAN_WR) | CCAN_ARB);
	CCAN_IF_Write(pCCAN, CMDREQ, IFsel, msg_num);
	while (CCAN_IF_Read(pCCAN, CMDREQ, IFsel) & CCAN_IFCREQ_BUSY) {}

}

/* Check the message objects is valid or not */
uint32_t IP_CCAN_GetValidMsg(IP_CCAN_001_T *pCCAN)
{
	return pCCAN->MSGV1 | (pCCAN->MSGV2 << 16);
}

/* Get the transmit repuest bit in all message objects */
uint32_t IP_CCAN_GetTxRQST(IP_CCAN_001_T *pCCAN)
{
	return pCCAN->TXREQ1 | (pCCAN->TXREQ2 << 16);
}

/* Set a message into the message object in message RAM */
void IP_CCAN_SetMsgObject(IP_CCAN_001_T *pCCAN,
						  IP_CCAN_MSG_INTERFACE_T IFsel,
						  uint8_t direction,
						  uint32_t RemoteEnable,
						  uint8_t msg_num,
						  const message_object *msg_ptr)
{
	uint16_t *buff_data;
	uint32_t msg_ctrl = 0;
	if (msg_ptr == NULL) {
		return;
	}
	buff_data = (uint16_t *) (msg_ptr->data);

	if (direction == CCAN_TX_DIR) {
		msg_ctrl |= CCAN_UMSK | CCAN_TXIE | CCAN_RMTEN(RemoteEnable) | CCAN_EOB | (msg_ptr->dlc & CCAN_DLC_MASK);
		if (!RemoteEnable) {
			msg_ctrl |= CCAN_TXRQ;
		}
	}
	else {
		msg_ctrl |= CCAN_UMSK | CCAN_RXIE | CCAN_RMTEN(RemoteEnable) | CCAN_EOB | (msg_ptr->dlc & CCAN_DLC_MASK);
	}

	CCAN_IF_Write(pCCAN, MCTRL, IFsel, msg_ctrl);
	CCAN_IF_Write(pCCAN, DA1, IFsel, *buff_data++);	/* Lower two bytes of message pointer */
	CCAN_IF_Write(pCCAN, DA2, IFsel, *buff_data++);	/* Upper two bytes of message pointer */
	CCAN_IF_Write(pCCAN, DB1, IFsel, *buff_data++);	/* Lower two bytes of message pointer */
	CCAN_IF_Write(pCCAN, DB2, IFsel, *buff_data);	/* Upper two bytes of message pointer */
	/* Configure arbitration */
	if (!(msg_ptr->id & (0x1 << 30))) {					/* bit 30 is 0, standard frame */
		/* Mxtd: 0, Mdir: 1, Mask is 0x7FF */
		CCAN_IF_Write(pCCAN, MSK2, IFsel, CCAN_MASK_MDIR(direction) | (CCAN_ID_STD_MASK << 2));
		CCAN_IF_Write(pCCAN, MSK1, IFsel, 0x0000);

		/* MsgVal: 1, Mtd: 0, Dir: 1, ID = 0x200 */
		CCAN_IF_Write(pCCAN, ARB2, IFsel, CCAN_ID_MVAL | CCAN_ID_DIR(direction) | (msg_ptr->id << 2));
		CCAN_IF_Write(pCCAN, ARB1, IFsel, 0x0000);
	}
	else {										/* Extended frame */
		/* Mxtd: 1, Mdir: 1, Mask is 0x1FFFFFFF */
		CCAN_IF_Write(pCCAN, MSK2, IFsel, CCAN_MASK_MXTD | CCAN_MASK_MDIR(direction) | (CCAN_ID_EXT_MASK >> 16));
		CCAN_IF_Write(pCCAN, MSK1, IFsel, CCAN_ID_EXT_MASK & 0x0000FFFF);

		/* MsgVal: 1, Mtd: 1, Dir: 1, ID = 0x200000 */
		CCAN_IF_Write(pCCAN, ARB2, IFsel, CCAN_ID_MVAL | CCAN_ID_MTD | CCAN_ID_DIR(direction) | (msg_ptr->id >> 16));
		CCAN_IF_Write(pCCAN, ARB1, IFsel, msg_ptr->id & 0x0000FFFF);
	}

	CCAN_IF_Buf_transfer(pCCAN, IFsel, msg_num, CCAN_WR);
}

/* Get a message object in message RAM into the message buffer */
void IP_CCAN_GetMsgObject(IP_CCAN_001_T *pCCAN, IP_CCAN_MSG_INTERFACE_T IFsel, uint8_t msg_num, message_object *msg_buf)
{
	uint32_t *temp_data;
	if (!msg_buf) {
		return;
	}
	temp_data = (uint32_t *) msg_buf->data;
	CCAN_IF_Buf_transfer(pCCAN, IFsel, msg_num, CCAN_RD);

	msg_buf->id = (CCAN_IF_Read(pCCAN, ARB1, IFsel) | (CCAN_IF_Read(pCCAN, ARB2, IFsel) << 16));
	msg_buf->dlc = CCAN_IF_Read(pCCAN, MCTRL, IFsel) & 0x000F;
	*temp_data++ = (CCAN_IF_Read(pCCAN, DA2, IFsel) << 16) | CCAN_IF_Read(pCCAN, DA1, IFsel);
	*temp_data = (CCAN_IF_Read(pCCAN, DB2, IFsel) << 16) | CCAN_IF_Read(pCCAN, DB1, IFsel);

	if (msg_buf->id & (0x1 << 30)) {
		msg_buf->id &= CCAN_ID_EXT_MASK;
	}
	else {
		msg_buf->id >>= 18;
		msg_buf->id &= CCAN_ID_STD_MASK;
	}
}
