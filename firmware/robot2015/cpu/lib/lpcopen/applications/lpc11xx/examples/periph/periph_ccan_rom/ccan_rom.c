/*
 * @brief CCAN on-chip driver example
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

/** @defgroup EXAMPLES_PERIPH_11XX_CCAN_ROM LPC11Cxx The CCAN on-chip drivers example
 * @ingroup EXAMPLES_PERIPH_11XX
 * <b>Example description</b><br>
 * This example shows how to use CCAN on-chip driver to send and receive a simple
 * CAN frame.<br>
 * Upon reset, the LPC11C00 will send a simple CAN frame with ID 0x345. It will
 * send a CAN frame with its data contents of "TEST".<br>
 * Whenever the LPC11C00 receives a CAN frame whose ID is between 0x400-0x4FF,
 * it will simply resend the received frame back onto the CAN bus.<br>
 * The baudrate is set to 500kBit/s.<br>
 *
 * <b>Special connection requirements</b><br>
 * Connect the CCAN_L and CCAN_H signal from the LPC11C00 to a CAN BUS Analyzer.<br>
 *
 * <b>Build procedures:</b><br>
 * @ref LPCOPEN_11XX_BUILDPROCS_XPRESSO<br>
 * @ref LPCOPEN_11XX_BUILDPROCS_KEIL<br>
 * @ref LPCOPEN_11XX_BUILDPROCS_IAR<br>
 *
 * <b>Supported boards and board setup:</b><br>
 * @ref LPCOPEN_11XX_BOARD_XPRESSO_11C24<br>
 *
 * <b>Submitting LPCOpen issues:</b><br>
 * @ref LPCOPEN_COMMUNITY
 * @{
 */

/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/

#define TEST_CCAN_BAUD_RATE 500000

CCAN_MSG_OBJ_T msg_obj;

/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/

/*****************************************************************************
 * Private functions
 ****************************************************************************/
void baudrateCalculate(uint32_t baud_rate, uint32_t *can_api_timing_cfg)
{
	uint32_t pClk, div, quanta, segs, seg1, seg2, clk_per_bit, can_sjw;
	Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_CAN);
	pClk = Chip_Clock_GetMainClockRate();

	clk_per_bit = pClk / baud_rate;

	for (div = 0; div <= 15; div++) {
		for (quanta = 1; quanta <= 32; quanta++) {
			for (segs = 3; segs <= 17; segs++) {
				if (clk_per_bit == (segs * quanta * (div + 1))) {
					segs -= 3;
					seg1 = segs / 2;
					seg2 = segs - seg1;
					can_sjw = seg1 > 3 ? 3 : seg1;
					can_api_timing_cfg[0] = div;
					can_api_timing_cfg[1] =
						((quanta - 1) & 0x3F) | (can_sjw & 0x03) << 6 | (seg1 & 0x0F) << 8 | (seg2 & 0x07) << 12;
					return;
				}
			}
		}
	}
}

/*	CAN receive callback */
/*	Function is executed by the Callback handler after
    a CAN message has been received */
void CAN_rx(uint8_t msg_obj_num) {
	/* Determine which CAN message has been received */
	msg_obj.msgobj = msg_obj_num;
	/* Now load up the msg_obj structure with the CAN message */
	LPC_CCAN_API->can_receive(&msg_obj);
	if (msg_obj_num == 1) {
		/* Simply transmit CAN frame (echo) with with ID +0x100 via buffer 2 */
		msg_obj.msgobj = 2;
		msg_obj.mode_id += 0x100;
		LPC_CCAN_API->can_transmit(&msg_obj);
	}
}

/*	CAN transmit callback */
/*	Function is executed by the Callback handler after
    a CAN message has been transmitted */
void CAN_tx(uint8_t msg_obj_num) {}

/*	CAN error callback */
/*	Function is executed by the Callback handler after
    an error has occured on the CAN bus */
void CAN_error(uint32_t error_info) {}

/**
 * @brief	CCAN Interrupt Handler
 * @return	Nothing
 * @note	The CCAN interrupt handler must be provided by the user application.
 *	It's function is to call the isr() API located in the ROM
 */
void CAN_IRQHandler(void) {
	LPC_CCAN_API->isr();
}

/*****************************************************************************
 * Public functions
 ****************************************************************************/

/**
 * @brief	Main routine for CCAN_ROM example
 * @return	Nothing
 */
int main(void)
{
	uint32_t CanApiClkInitTable[2];
	/* Publish CAN Callback Functions */
	CCAN_CALLBACKS_T callbacks = {
		CAN_rx,
		CAN_tx,
		CAN_error,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
	};
	Board_Init();
	baudrateCalculate(TEST_CCAN_BAUD_RATE, CanApiClkInitTable);

	LPC_CCAN_API->init_can(&CanApiClkInitTable[0], TRUE);
	/* Configure the CAN callback functions */
	LPC_CCAN_API->config_calb(&callbacks);
	/* Enable the CAN Interrupt */
	NVIC_EnableIRQ(CAN_IRQn);

	/* Send a simple one time CAN message */
	msg_obj.msgobj  = 0;
	msg_obj.mode_id = 0x345;
	msg_obj.mask    = 0x0;
	msg_obj.dlc     = 4;
	msg_obj.data[0] = 'T';	// 0x54
	msg_obj.data[1] = 'E';	// 0x45
	msg_obj.data[2] = 'S';	// 0x53
	msg_obj.data[3] = 'T';	// 0x54
	LPC_CCAN_API->can_transmit(&msg_obj);

	/* Configure message object 1 to receive all 11-bit messages 0x400-0x4FF */
	msg_obj.msgobj = 1;
	msg_obj.mode_id = 0x400;
	msg_obj.mask = 0x700;
	LPC_CCAN_API->config_rxmsgobj(&msg_obj);

	while (1) {
		__WFI();	/* Go to Sleep */
	}
}

/**
 * @}
 */
