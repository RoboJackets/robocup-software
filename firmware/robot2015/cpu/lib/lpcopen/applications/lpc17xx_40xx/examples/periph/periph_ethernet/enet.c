/*
 * @brief	Simple ethernet example
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
#include <stdio.h>
#include <string.h>

/** @defgroup EXAMPLES_PERIPH_17XX40XX_ENET LPC17xx/40xx Ethernet example
 * @ingroup EXAMPLES_PERIPH_17XX40XX
 * <b>Example description</b><br>
 * The ENET example is a simple ethernet application for sending an ethernet
 * packet via the MAC and displaying received broadcast packets. The MAC example will
 * listen for all packets on the ethernet and display some stats via UART when a
 * packet is received. A dummy packet can also be sent out. Wireshark can be used to
 * view the outgoing packet.<br>
 *
 * To use the example, connect a serial cable to the board's RS232/UART port and
 * start a terminal program to monitor the port.  The terminal program on the host
 * PC should be setup for 115K8N1. For each packet received, the LED will toggle
 * and the packets source and destination MAC addresses will be displayed with the
 * packet type. Pressing any key will send a dummy packet.<br>
 *
 * <b>Special connection requirements</b><br>
 * - Embedded Artists' LPC1788 Developer's Kit:<br>
 * - Embedded Artists' LPC4088 Developer's Kit:<br>
 * There are no special connection requirements for this example.<br>
 * - LPCXpresso LPC1769:<br>
 * Need to connect with base board for using RS232/UART port and Ethernet port.<br>
 *
 * <b>Build procedures:</b><br>
 * @ref LPCOPEN_17XX40XX_BUILDPROCS_KEIL<br>
 * @ref LPCOPEN_17XX40XX_BUILDPROCS_IAR<br>
 * @ref LPCOPEN_17XX40XX_BUILDPROCS_XPRESSO<br>
 *
 * <b>Supported boards and board setup:</b><br>
 * @ref LPCOPEN_17XX40XX_BOARD_EA1788<br>
 * @ref LPCOPEN_17XX40XX_BOARD_EA4088<br>
 * @ref LPCOPEN_17XX40XX_BOARD_XPRESSO_1769<br>
 *
 * <b>Submitting LPCOpen issues:</b><br>
 * @ref LPCOPEN_COMMUNITY
 * @{
 */

/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/

#define ENET_NUM_TX_DESC 4
#define ENET_NUM_RX_DESC 4

#if defined(CHIP_LPC175X_6X)
#define ENET_RX_DESC_BASE        (0x2007C000)
#else
/* The Ethernet Block can only access Peripheral SRAM and External Memory. In this example,
   Peripheral SRAM is selected for storing descriptors, status arrays and send/receive buffers.*/
#define ENET_RX_DESC_BASE        (0x20000000UL)
#endif
#define ENET_RX_STAT_BASE        (ENET_RX_DESC_BASE + ENET_NUM_RX_DESC * sizeof(ENET_RXDESC_T))
#define ENET_TX_DESC_BASE        (ENET_RX_STAT_BASE + ENET_NUM_RX_DESC * sizeof(ENET_RXSTAT_T))
#define ENET_TX_STAT_BASE        (ENET_TX_DESC_BASE + ENET_NUM_TX_DESC * sizeof(ENET_TXDESC_T))
#define ENET_RX_BUF_BASE         (ENET_TX_STAT_BASE + ENET_NUM_TX_DESC * sizeof(ENET_TXSTAT_T))
#define ENET_TX_BUF_BASE         (ENET_RX_BUF_BASE  + ENET_NUM_RX_DESC * ENET_ETH_MAX_FLEN)
#define ENET_RX_BUF(i)           (ENET_RX_BUF_BASE + ENET_ETH_MAX_FLEN * i)
#define ENET_TX_BUF(i)           (ENET_TX_BUF_BASE + ENET_ETH_MAX_FLEN * i)

STATIC ENET_RXDESC_T *pRXDescs = (ENET_RXDESC_T *) ENET_RX_DESC_BASE;
STATIC ENET_RXSTAT_T *pRXStats = (ENET_RXSTAT_T *) ENET_RX_STAT_BASE;
STATIC ENET_TXDESC_T *pTXDescs = (ENET_TXDESC_T *) ENET_TX_DESC_BASE;
STATIC ENET_TXSTAT_T *pTXStats = (ENET_TXSTAT_T *) ENET_TX_STAT_BASE;

/* Transmit/receive buffers and indices */
STATIC int32_t rxConsumeIdx;
STATIC int32_t txProduceIdx;

/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/

/*****************************************************************************
 * Private functions
 ****************************************************************************/

/* Local delay function used by the ENET or PHY drivers . This can be
   replaced with something more accurate if needed. */
STATIC void localMsDelay(uint32_t ms)
{
	ms = ms * 40000;
	while (ms > 0) {
		ms--;
	}
}

/* Initialize MAC descriptors for simple packet receive/transmit */
STATIC void InitDescriptors(void)
{
	int i;

	/* Setup the descriptor list to a default state */
	memset(pTXDescs, 0, ENET_NUM_TX_DESC * sizeof(ENET_TXDESC_T));
	memset(pTXStats, 0, ENET_NUM_TX_DESC * sizeof(ENET_TXSTAT_T));
	memset(pRXDescs, 0, ENET_NUM_RX_DESC * sizeof(ENET_RXDESC_T));
	memset(pRXStats, 0, ENET_NUM_RX_DESC * sizeof(ENET_RXSTAT_T));

	rxConsumeIdx = 0;
    rxConsumeIdx = 0;

	/* Build linked list, CPU is owner of descriptors */
	for (i = 0; i < ENET_NUM_RX_DESC; i++) {
		pRXDescs[i].Packet = (uint32_t) ENET_RX_BUF(i);
		pRXDescs[i].Control = ENET_RCTRL_SIZE(ENET_ETH_MAX_FLEN);
		pRXStats[i].StatusInfo = 0;
		pRXStats[i].StatusHashCRC = 0;
	}
	for (i = 0; i < ENET_NUM_TX_DESC; i++) {
		pTXDescs[i].Packet = (uint32_t) ENET_TX_BUF(i);
		pTXDescs[i].Control = 0;
		pTXStats[i].StatusInfo = 0;
	}

	/* Setup list pointers in Ethernet controller */
	Chip_ENET_InitTxDescriptors(LPC_ETHERNET, pTXDescs, pTXStats, ENET_NUM_TX_DESC);
	Chip_ENET_InitRxDescriptors(LPC_ETHERNET, pRXDescs, pRXStats, ENET_NUM_RX_DESC);
}

/* Get the pointer to the Rx buffer storing new received frame */
STATIC void *ENET_RXGet(int32_t *bytes)
{
	uint16_t produceIdx;
	void *buffer;

	produceIdx = Chip_ENET_GetRXProduceIndex(LPC_ETHERNET);
	/* This doesn't check status of the received packet */
	if (Chip_ENET_GetBufferStatus(LPC_ETHERNET, produceIdx, rxConsumeIdx, ENET_NUM_RX_DESC) != ENET_BUFF_EMPTY) {
		/* CPU owns descriptor, so a packet was received */
		buffer = (void *) pRXDescs[rxConsumeIdx].Packet;
		*bytes = (int32_t) (ENET_RINFO_SIZE(pRXStats[rxConsumeIdx].StatusInfo) - 4);/* Remove CRC */
	}
	else {
		/* Nothing received */
		*bytes = 0;
		buffer = NULL;
	}

	return buffer;
}

/* Release Rx Buffer */
STATIC void ENET_RXBuffClaim(void)
{
	rxConsumeIdx = Chip_ENET_IncRXConsumeIndex(LPC_ETHERNET);
}

/* Get Tx Buffer for the next transmission */
STATIC void *ENET_TXBuffGet(void)
{
	uint16_t consumeIdx = Chip_ENET_GetTXConsumeIndex(LPC_ETHERNET);

	if (Chip_ENET_GetBufferStatus(LPC_ETHERNET, txProduceIdx, consumeIdx, ENET_NUM_TX_DESC) != ENET_BUFF_FULL) {
		return (void *) pTXDescs[txProduceIdx].Packet;
	}
	return NULL;
}

/* Queue a new frame for transmission */
STATIC void ENET_TXQueue(int32_t bytes)
{
	if (bytes > 0) {
		pTXDescs[txProduceIdx].Control = ENET_TCTRL_SIZE(bytes) | ENET_TCTRL_LAST;
		txProduceIdx = Chip_ENET_IncTXProduceIndex(LPC_ETHERNET);
	}
}

/* Check if tranmission finished */
STATIC bool ENET_IsTXFinish(void)
{
	uint16_t consumeIdx = Chip_ENET_GetTXConsumeIndex(LPC_ETHERNET);

	if (Chip_ENET_GetBufferStatus(LPC_ETHERNET, txProduceIdx, consumeIdx, ENET_NUM_TX_DESC) == ENET_BUFF_EMPTY) {
		return true;
	}
	return false;
}

/*****************************************************************************
 * Public functions
 ****************************************************************************/

/**
 * @brief	Handle interrupt from ethernet
 * @return	Nothing
 */
void ETH_IRQHandler(void)
{
	/* This demo is entirely polled, so the IRQ isn't really needed */
}

/**
 * @brief	main routine for ENET example
 * @return	Nothing (function should not exit)
 */
int main(void)
{
	uint8_t macaddr[6], *workbuff;
	uint32_t physts = 0;
	int32_t rxBytes, i;
	bool ethpkttgl = true;
	volatile int k = 1;

	/* LED0 is used for the link status, on = PHY cable detected */
	Board_Init();

	/* Initial LED state is off to show an unconnected cable state */
	Board_LED_Set(0, false);

	/* Setup ethernet and PHY */
	Chip_ENET_Init(LPC_ETHERNET);

	/* Setup MII clock rate and PHY address */
	Chip_ENET_SetupMII(LPC_ETHERNET, Chip_ENET_FindMIIDiv(LPC_ETHERNET, 2500000), 1);

	lpc_phy_init(true, localMsDelay);

	/* Setup MAC address for device */
	Board_ENET_GetMacADDR(macaddr);
	Chip_ENET_SetADDR(LPC_ETHERNET, macaddr);

	/* Setup descriptors */
	InitDescriptors();

	/* Enable RX/TX after descriptors are setup */
	Chip_ENET_TXEnable(LPC_ETHERNET);
	Chip_ENET_RXEnable(LPC_ETHERNET);

	while (k) {
		/* Check for receive packets */
		workbuff = ENET_RXGet(&rxBytes);
		if (workbuff) {
			/* Packet was received. Dump some info from the packet */
			DEBUGOUT("Packet received: ");
			ethpkttgl = (bool) !ethpkttgl;

			/* Toggle LED2 on each packet received */
			Board_LED_Set(1, ethpkttgl);

			/* Destination and source MAC addresses */
			DEBUGOUT("-Dest MAC: %02x:%02x:%02x:%02x:%02x:%02x-",
					 workbuff[0], workbuff[1], workbuff[2], workbuff[3], workbuff[4], workbuff[5]);
			DEBUGOUT("-Source MAC: %02x:%02x:%02x:%02x:%02x:%02x-",
					 workbuff[6], workbuff[7], workbuff[8], workbuff[9], workbuff[10], workbuff[11]);

			/* Length of received packet and embedded len/type */
			DEBUGOUT("-Packet len: %d", rxBytes);
			DEBUGOUT("-Type: %04x\r\n", ((((uint32_t) workbuff[12]) << 8) | (((uint32_t) workbuff[13]))));
			ENET_RXBuffClaim();
		}

		/* Send a 'dummy' broadcast packet on a keypress. You'll only see this
		     if your using a tool such as WireShark and the packet will not have
		   a checksum */
		if (DEBUGIN() != EOF) {
			/* Only if link detected */
			if (physts & PHY_LINK_CONNECTED) {
				workbuff = ENET_TXBuffGet();
				if (workbuff) {
					/* Destination is broadcast */
					for (i = 0; i < 6; i++) {
						workbuff[i] = 0xFF;
					}

					/* Source is this MAC address */
					memcpy(&workbuff[6], macaddr, 6);

					/* Size will be 128 bytes (total) */
					workbuff[12] = 0;
					workbuff[13] = 128;

					/* Some dummy data, fill beyond end of packet */
					for (i = 0; i < 128; i++) {
						workbuff[i + 14] = (uint8_t) (i & 0xFF);
					}

					ENET_TXQueue(128);
					DEBUGOUT("Packet sent! \r\n");
					Board_LED_Set(1, true);
				}
			}
		}
		if (ENET_IsTXFinish()) {
			Board_LED_Set(1, false);
		}
		/* PHY status state machine, LED on when connected. This function will
		   not block. */
		physts = lpcPHYStsPoll();

		/* Only check for connection state when the PHY status has changed */
		if (physts & PHY_LINK_CHANGED) {
			if (physts & PHY_LINK_CONNECTED) {
				Board_LED_Set(0, true);

				/* Set interface speed and duplex */
				if(physts & PHY_LINK_FULLDUPLX)
					Chip_ENET_SetFullDuplex(LPC_ETHERNET);
				else
					Chip_ENET_SetHalfDuplex(LPC_ETHERNET);
				if(physts & PHY_LINK_SPEED100)
					Chip_ENET_Set100Mbps(LPC_ETHERNET);
				else
					Chip_ENET_Set10Mbps(LPC_ETHERNET);
			}
			else {
				Board_LED_Set(0, false);
			}

			DEBUGOUT("Link connect status: %d\r\n", ((physts & PHY_LINK_CONNECTED) != 0));
		}
	}

	/* Never returns, for warning only */
	return 0;
}

/**
 * @}
 */
