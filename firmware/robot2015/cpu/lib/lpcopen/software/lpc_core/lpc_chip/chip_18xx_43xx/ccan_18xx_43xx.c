/*
 * @brief LPC18xx/43xx CCAN driver
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

#include "chip.h"

/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/

#define MAX_OBJECT 32
#define CHIP_CCAN_DETERMINECLK(n) (((n) == LPC_C_CAN0) ? CLK_APB3_CAN0 : CLK_APB1_CAN1)

/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/

/*****************************************************************************
 * Private functions
 ****************************************************************************/

/* Return 1->32; 0 if not find free msg */
static uint8_t getFreeMsgObject(LPC_CCAN_T *pCCAN)
{
	uint32_t msg_valid;
	uint8_t i;
	msg_valid = IP_CCAN_GetValidMsg(pCCAN);
	for (i = 0; i < MAX_OBJECT; i++) {
		if (!((msg_valid >> i) & 1UL)) {
			return i + 1;
		}
	}
	return 0;	// No free object
}

static void Free_msg_object(LPC_CCAN_T *pCCAN, uint8_t msg_num)
{
	IP_CCAN_SetValidMsg(pCCAN, IF1, msg_num, DISABLE);
}

/*****************************************************************************
 * Public functions
 ****************************************************************************/

/* Select bit rate for CCAN bus */
void Chip_CCAN_SetBitRate(LPC_CCAN_T *pCCAN, uint32_t bitRate)
{
	uint32_t pClk, clk_div = 1, div, quanta, segs, seg1, seg2, clk_per_bit, can_sjw;
	pClk = Chip_Clock_GetRate(CHIP_CCAN_DETERMINECLK(pCCAN));
	clk_per_bit = pClk / bitRate;

	for (div = 0; div <= 15; div++) {
		if (div) {
			clk_div = (1 << (div - 1)) + 1;
		}
		for (quanta = 1; quanta <= 32; quanta++) {
			for (segs = 3; segs <= 17; segs++) {
				if (clk_per_bit == (segs * quanta * clk_div)) {
					segs -= 3;
					seg1 = segs / 2;
					seg2 = segs - seg1;
					can_sjw = seg1 > 3 ? 3 : seg1;
					IP_CCAN_TimingCfg(pCCAN, div, quanta - 1, can_sjw, seg1, seg2);
					return;
				}
			}
		}
	}
	while (1) {	// Can not find a correct value with input bitrate
	}
}

/* Send a message */
void Chip_CCAN_Send(LPC_CCAN_T *pCCAN, uint32_t RemoteEnable, message_object *msg_ptr)
{
	uint8_t msg_num_send = getFreeMsgObject(pCCAN);
	if (!msg_num_send) {
		return;
	}
	IP_CCAN_SetMsgObject(pCCAN, IF1, CCAN_TX_DIR, RemoteEnable, msg_num_send, msg_ptr);
	while (IP_CCAN_GetTxRQST(pCCAN) >> (msg_num_send - 1)) {	// blocking , wait for sending completed
	}
	if (!RemoteEnable) {
		Free_msg_object(pCCAN, msg_num_send);
	}
}

/* Initialize the CCAN peripheral, free all message object in RAM */
void Chip_CCAN_Init(LPC_CCAN_T *pCCAN)
{
	uint8_t i;
	uint32_t can_stat;

	Chip_Clock_EnableOpts(CHIP_CCAN_DETERMINECLK(pCCAN), true, false, 1);
	can_stat = IP_CCAN_GetStatus(pCCAN);
	for (i = 1; i <= MAX_OBJECT; i++) {
		Free_msg_object(pCCAN, i);
	}
	IP_CCAN_SetStatus(pCCAN, can_stat & (~(CCAN_STAT_RXOK | CCAN_STAT_TXOK)));
}

/* De-initialize the CCAN peripheral */
void Chip_CCAN_DeInit(LPC_CCAN_T *pCCAN)
{
	// FIXME is any specific CAN shutdown code needed here?
	Chip_Clock_Disable(CHIP_CCAN_DETERMINECLK(pCCAN));
}

/* Register a message ID for receiving */
void Chip_CCAN_AddReceiveID(LPC_CCAN_T *pCCAN, uint32_t rev_id)
{
	message_object temp;
	uint8_t msg_num_rev = getFreeMsgObject(pCCAN);
	if (!msg_num_rev) {
		return;
	}
	temp.id = rev_id;
	IP_CCAN_SetMsgObject(pCCAN, IF2, CCAN_RX_DIR, 0, msg_num_rev, &temp);
}

/* Remove a registered message ID from receiving */
void Chip_CCAN_DeleteReceiveID(LPC_CCAN_T *pCCAN, uint32_t rev_id)
{
	uint8_t i;
	message_object temp;
	for (i = 1; i <= MAX_OBJECT; i++) {
		IP_CCAN_GetMsgObject(pCCAN, IF1, i, &temp);
		if (temp.id == rev_id) {
			Free_msg_object(pCCAN, i);
		}
	}
}

/* Clear the pending interrupt */
void Chip_CCAN_ClearIntPend(LPC_CCAN_T *pCCAN, uint8_t msg_num, uint8_t TRxMode)
{
	uint32_t can_stat = IP_CCAN_GetStatus(pCCAN);
	if (TRxMode == CCAN_TX_DIR) {
		IP_CCAN_SetStatus(pCCAN, can_stat & (~CCAN_STAT_TXOK));
	}
	else {
		IP_CCAN_SetStatus(pCCAN, can_stat & (~CCAN_STAT_RXOK));
	}
	IP_CCAN_ClearIntPend(pCCAN, IF1, msg_num);

}

/* Clear the status of CCAN bus */
void Chip_CCAN_ClearStatus(LPC_CCAN_T *pCCAN, IP_CCAN_STATUS_T status)
{
	uint32_t tmp = IP_CCAN_GetStatus(pCCAN);
	IP_CCAN_SetStatus(pCCAN, tmp & (~status));
}
