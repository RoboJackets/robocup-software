/*
 * @brief	Simple ethernet example
 *			This simple MAC example will listen for all packets on
 *			the ethernet and display some stats via UART when a
 *			packet is received. A dummy packet can also be sent out.
 *			Wireshark can be used to view the outgoing packet.
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

/** @defgroup EXAMPLES_PERIPH_18XX43XX_ENET LPC18xx/43xx Ethernet example
 * @ingroup EXAMPLES_PERIPH_18XX43XX
 * <b>Example description</b><br>
 * The ENET example is a simple Ethernet application for sending an Ethernet
 * packet via the MAC and displaying received broadcast packets.<br>
 *
 * To use the example, connect a serial cable to the board's RS232/UART port and
 * start a terminal program to monitor the port.  The terminal program on the host
 * PC should be setup for 115.2K 8N1. For each packet received, the LED will toggle
 * and the packets source and destination MAC addresses will be displayed with the
 * packet type. Pressing any key will send a dummy packet.<br>
 *
 * <b>Special connection requirements</b><br>
 * There are no special connection requirements for this example.<br>
 *
 * <b>Build procedures:</b><br>
 * @ref LPCOPEN_18XX43XX_BUILDPROCS_KEIL<br>
 * @ref LPCOPEN_18XX43XX_BUILDPROCS_IAR<br>
 * @ref LPCOPEN_18XX43XX_BUILDPROCS_XPRESSO<br>
 *
 * <b>Supported boards and board setup:</b><br>
 * @ref LPCOPEN_18XX_BOARD_HITEX1850<br>
 * @ref LPCOPEN_43XX_BOARD_HITEX4350<br>
 * @ref LPCOPEN_18XX_BOARD_KEIL1857<br>
 * @ref LPCOPEN_43XX_BOARD_KEIL4357<br>
 * @ref LPCOPEN_18XX_BOARD_NGX1830<br>
 * @ref LPCOPEN_43XX_BOARD_NGX4330<br>
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

static IP_ENET_001_ENHTXDESC_T TXDescs[ENET_NUM_TX_DESC];
static IP_ENET_001_ENHRXDESC_T RXDescs[ENET_NUM_RX_DESC];

/* Transmit/receive buffers and indices */
static uint8_t TXBuffer[ENET_NUM_TX_DESC][EMAC_ETH_MAX_FLEN];
static uint8_t RXBuffer[ENET_NUM_RX_DESC][EMAC_ETH_MAX_FLEN];
static int32_t rxFill, rxGet, rxAvail, rxNumDescs;
static int32_t txFill, txGet, txUsed, txNumDescs;

/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/

/*****************************************************************************
 * Private functions
 ****************************************************************************/

/* Local delay function used by the ENET or PHY drivers . This can be
   replaced with something more accurate if needed. */
static void localMsDelay(uint32_t ms)
{
	ms = ms * 50000;
	while (ms > 0) {
		ms--;
	}
}

/* Local index and check function */
static __INLINE int32_t incIndex(int32_t index, int32_t max)
{
	index++;
	if (index >= max) {
		index = 0;
	}

	return index;
}

/* Initialize MAC descriptors for simple packet receive/transmit */
static void InitDescriptors(
	IP_ENET_001_ENHTXDESC_T *pTXDescs, int32_t numTXDescs,
	IP_ENET_001_ENHRXDESC_T *pRXDescs, int32_t numRXDescs)
{
	int i;

	/* Setup the descriptor list to a default state */
	memset(pTXDescs, 0, numTXDescs * sizeof(*pTXDescs));
	memset(pTXDescs, 0, numRXDescs * sizeof(*pRXDescs));
	rxFill = rxGet = 0;
	rxAvail = rxNumDescs = numRXDescs;
	txNumDescs = numTXDescs;
	txUsed = txGet = txFill = 0;

	/* Build linked list, CPU is owner of descriptors */
	for (i = 0; i < numTXDescs; i++) {
		pTXDescs[i].CTRLSTAT = 0;
		pTXDescs[i].B2ADD = (uint32_t) &pTXDescs[i + 1];
	}
	pTXDescs[numTXDescs - 1].B2ADD = (uint32_t) &pTXDescs[0];
	for (i = 0; i < numRXDescs; i++) {
		pRXDescs[i].STATUS = 0;
		pRXDescs[i].B2ADD = (uint32_t) &pRXDescs[i + 1];
		pRXDescs[i].CTRL = RDES_ENH_RCH;
	}
	pRXDescs[numRXDescs - 1].B2ADD = (uint32_t) &pRXDescs[0];
	pRXDescs[numRXDescs - 1].CTRL |= RDES_ENH_RER;

	/* Setup list pointers in Ethernet controller */
	Chip_ENET_InitDescriptors(LPC_ETHERNET, pTXDescs, pRXDescs);
}

/* Attach a buffer to a descriptor and queue it for reception */
static void ENET_RXQueue(void *buffer, int32_t bytes)
{
	if (rxAvail > 0) {
		/* Queue the next descriptor and start polling */
		RXDescs[rxFill].B1ADD = (uint32_t) buffer;
		RXDescs[rxFill].CTRL = RDES_ENH_BS1(bytes) | RDES_ENH_RCH;
		if (rxFill == (rxNumDescs - 1)) {
			RXDescs[rxFill].CTRL |= RDES_ENH_RER;
		}
		RXDescs[rxFill].STATUS = RDES_OWN;
		rxAvail--;
		rxFill = incIndex(rxFill, rxNumDescs);

		/* Start polling */
		Chip_ENET_RXStart(LPC_ETHERNET);
	}
}

/* Returns a pointer to a filled ethernet buffer or NULL if none are available */
static void *ENET_RXGet(int32_t *bytes)
{
	void *buffer;

	/* This doesn't check status of the received packet */
	if ((rxAvail < rxNumDescs) && (!(RXDescs[rxGet].STATUS & RDES_OWN))) {
		/* CPU owns descriptor, so a packet was received */
		buffer = (void *) RXDescs[rxGet].B1ADD;
		*bytes = (int32_t) RXDescs[rxGet].STATUS & 0xFFF;
		rxGet = incIndex(rxGet, rxNumDescs);
		rxAvail++;
	}
	else {
		/* Nothing received */
		*bytes = 0;
		buffer = NULL;
	}

	return buffer;
}

/* Attaches a buffer to a transmit descriptor and queues it for transmit */
static void ENET_TXQueue(void *buffer, int32_t bytes)
{
	if (txUsed < txNumDescs) {
		/* Queue the next descriptor and start polling */
		TXDescs[txFill].B1ADD = (uint32_t) buffer;
		TXDescs[txFill].BSIZE = TDES_ENH_BS1(bytes);
		TXDescs[txFill].CTRLSTAT = TDES_ENH_FS | TDES_ENH_LS | TDES_ENH_TCH;
		if (txFill == (txNumDescs - 1)) {
			TXDescs[txFill].CTRLSTAT |= TDES_ENH_TER;
		}
		TXDescs[txFill].CTRLSTAT |= TDES_OWN;
		txUsed++;
		txFill = incIndex(txFill, txNumDescs);

		/* Start polling */
		Chip_ENET_TXStart(LPC_ETHERNET);
	}
}

/* Returns a pointer to a buffer that has been transmitted */
static void *ENET_TXBuffClaim(void)
{
	void *buffer;

	/* Is packet done sending? */
	if ((txUsed > 0) && (!(TXDescs[txGet].CTRLSTAT & TDES_OWN))) {
		/* CPU owns descriptor, so the packet completed transmit */
		buffer = (void *) TXDescs[txGet].B1ADD;
		txGet = incIndex(txGet, txNumDescs);
		txUsed--;
		Board_LED_Set(2, false);
	}
	else {
		buffer = NULL;
	}

	return buffer;
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
	int32_t rxBytes, i, txNextIndex;
	bool ethpkttgl = true;

	/* LED0 is used for the link status, on = PHY cable detected */
	Board_Init();

	/* Initial LED state is off to show an unconnected cable state */
	Board_LED_Set(0, false);

	/* Setup ethernet and PHY */
	Chip_ENET_Init(LPC_ETHERNET);
#if defined(USE_RMII)
	lpc_phy_init(true, localMsDelay);
#else
	lpc_phy_init(false, localMsDelay);
#endif

	/* Setup MAC address for device */
	Board_ENET_GetMacADDR(macaddr);
	Chip_ENET_SetADDR(LPC_ETHERNET, macaddr);

	/* Setup descriptors */
	InitDescriptors(TXDescs, ENET_NUM_TX_DESC, RXDescs, ENET_NUM_RX_DESC);

	/* Attach a buffer to a RX descriptor and queue it for receive */
	i = 0;
	while (i < ENET_NUM_RX_DESC) {
		ENET_RXQueue(RXBuffer[i], EMAC_ETH_MAX_FLEN);
		i++;
	}
	txNextIndex = 0;

	/* Enable RX/TX after descriptors are setup */
	Chip_ENET_TXEnable(LPC_ETHERNET);
	Chip_ENET_RXEnable(LPC_ETHERNET);

	while (1) {
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

			/* Re-queue the (same) packet again */
			ENET_RXQueue(workbuff, EMAC_ETH_MAX_FLEN);
		}

		/* Send a 'dummy' broadcast packet on a keypress. You'll only see this
		     if your using a tool such as WireShark and the packet will not have
		   a checksum */
		if (DEBUGIN() != EOF) {
			/* Only if link detected */
			if (physts & PHY_LINK_CONNECTED) {
				/* Get next available TX buffer */
				workbuff = (uint8_t *) TXBuffer[txNextIndex];
				txNextIndex = incIndex(txNextIndex, ENET_NUM_TX_DESC);
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

				/* Queue TX packet. This is an Ethernet packet, but the payload and
				   hecksum are garbage. The packet will still be viewable with
				   Wireshark */
				ENET_TXQueue(workbuff, 128);
				Board_LED_Set(2, true);
			}
		}

		/* Transmit buffers are 'zero-copy' buffers with the queue function, so
		   the buffer must remain in memory until the packet has been fully
		   transmitted. Call ENET_TXBuffClaim() to to reclaim a sent
		   packet. If a packet buffer address is return (not NULL), then the
		   packet can be de-allocated. Since the buffers in this examples are
		   static, there isn't too much to do. */
		ENET_TXBuffClaim();

		/* PHY status state machine, LED on when connected. This function will
		   not block. */
		physts = lpcPHYStsPoll();

		/* Only check for connection state when the PHY status has changed */
		if (physts & PHY_LINK_CHANGED) {
			if (physts & PHY_LINK_CONNECTED) {
				Board_LED_Set(0, true);

				/* Set interface speed and duplex */
				Chip_ENET_SetDuplex(LPC_ETHERNET, (bool) (physts & PHY_LINK_FULLDUPLX));
				Chip_ENET_SetSpeed(LPC_ETHERNET, (bool) (physts & PHY_LINK_SPEED100));
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
