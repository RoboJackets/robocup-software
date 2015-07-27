/*
 * @brief CCAN example
 * This example show how to  the CCAN in 3 mode : Polling, Interrupt and DMA
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

#define CCAN_TX_MSG_ID (0x200)
#define CCAN_RX_MSG_ID (0x100)
#define CCAN_TX_MSG_REMOTE_ID (0x300)

// FIXME - needs formatting to correct standards

#include "board.h"
static char WelcomeMenu[] = "\n\rHello NXP Semiconductors \r\r"
							"CCAN DEMO : Use C_CAN to transmit and receive Message from CAN Analyzer\r\n"
							"CCAN bit rate : 500kBit/s\r\n";

uint8_t msg_received_counter = 0;

void CAN0_IRQHandler(void)
{
	message_object msg_buf;
	uint32_t can_int, can_stat, i;
	while ( (can_int = Chip_CCAN_GetIntID(LPC_C_CAN0)) != 0 ) {
		if (can_int == CCAN_STATUS_INT) {
			can_stat = Chip_CCAN_GetStatus(LPC_C_CAN0);
			// TODO with error or TXOK, RXOK
			if (can_stat & CCAN_STAT_EPASS) {
				DEBUGOUT("Passive error\r\n");
				return;
			}
			if (can_stat & CCAN_STAT_EWARN) {
				DEBUGOUT("Warning!!!\r\n");
				return;
			}
			if (can_stat & CCAN_STAT_BOFF) {
				DEBUGOUT("CAN bus is off\r\n");
				return;
			}
			Chip_CCAN_ClearStatus(LPC_C_CAN0, CCAN_STAT_TXOK);
			Chip_CCAN_ClearStatus(LPC_C_CAN0, CCAN_STAT_RXOK);
		}
		else if ((1 <= can_int) && (can_int <= 0x20)) {
			// Process msg num canint
			Chip_CCAN_GetMsgObject(LPC_C_CAN0, can_int, &msg_buf);
			switch (msg_buf.id) {
			case CCAN_RX_MSG_ID:
				Chip_CCAN_ClearIntPend(LPC_C_CAN0, can_int, CCAN_RX_DIR);
				DEBUGOUT("Msg ID :%x\r\n", msg_buf.id);
				DEBUGOUT("Msg data :");
				for (i = 0; i < msg_buf.dlc; i++)
					DEBUGOUT("%x ", msg_buf.data[i]);
				DEBUGOUT("\nFeed back...\r\n");
				msg_buf.id += 1;
				Chip_CCAN_Send(LPC_C_CAN0, 0, &msg_buf);
				break;

			case CCAN_TX_MSG_ID:
				Chip_CCAN_ClearIntPend(LPC_C_CAN0, can_int, CCAN_TX_DIR);
				break;

			case CCAN_TX_MSG_REMOTE_ID:
				msg_received_counter++;
				if (msg_received_counter == 5) {
					DEBUGOUT("Remote transmit total is 5. Delete remote ID\r\n");
					Chip_CCAN_DeleteReceiveID(LPC_C_CAN0, CCAN_TX_MSG_REMOTE_ID);
				}
				break;

			default:
				break;
			}
		}
	}
}

int main(void)
{
	message_object send_obj;
	Board_Init();
	DEBUGOUT(WelcomeMenu);
	 /* Set CCAN peripheral clock under 100Mhz for working stable */
	Chip_Clock_SetBaseClock(CLK_BASE_APB3, CLKIN_IDIVC, true, false);
	Chip_CCAN_Init(LPC_C_CAN0);
	Chip_CCAN_SetBitRate(LPC_C_CAN0, 500000);
	Chip_CCAN_IntEnable(LPC_C_CAN0, ENABLE);

	send_obj.id = CCAN_TX_MSG_ID;
	send_obj.dlc = 4;
	send_obj.data[0] = 'A';
	send_obj.data[1] = 'B';
	send_obj.data[2] = 'C';
	send_obj.data[3] = 'D';
	Chip_CCAN_Send(LPC_C_CAN0, 0, &send_obj);
	
	send_obj.id = CCAN_TX_MSG_REMOTE_ID;
	send_obj.data[0] = 'E';
	send_obj.data[1] = 'F';
	send_obj.data[2] = 'G';
	send_obj.data[3] = 'H';
	Chip_CCAN_Send(LPC_C_CAN0, 1, &send_obj);

	Chip_CCAN_AddReceiveID(LPC_C_CAN0, CCAN_RX_MSG_ID);

	NVIC_EnableIRQ(C_CAN0_IRQn);

	while (1);
}
