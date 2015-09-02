/*
 * @brief Cyclic Redundancy Check (CRC) generator example.
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

/** @defgroup EXAMPLES_PERIPH_17XX40XX_CRC LPC17xx/40xx Cyclic Redundancy Check (CRC) generator example
 * @ingroup EXAMPLES_PERIPH_17XX40XX
 * <b>Example description</b><br>
 * The CRC example demonstrates using the CRC engine for 8-bit, 16-bit, and
 * 32-bit CRC computation. The CRC engine will run via CRC computations for 8-bit,
 * 16-bit, and 32-bit blocks and verify the CRC checksum. The expected and actual
 * results of the test are output on the UART. If the test fails, the LED will
 * be on.
 *
 * <b>Special connection requirements</b><br>
 * There are no special connection requirements for this example.<br>
 *
 * <b>Build procedures:</b><br>
 * @ref LPCOPEN_17XX40XX_BUILDPROCS_KEIL<br>
 * @ref LPCOPEN_17XX40XX_BUILDPROCS_IAR<br>
 * @ref LPCOPEN_17XX40XX_BUILDPROCS_XPRESSO<br>
 *
 * <b>Supported boards and board setup:</b><br>
 * @ref LPCOPEN_17XX40XX_BOARD_EA1788<br>
 * @ref LPCOPEN_17XX40XX_BOARD_EA4088<br>
 *
 * <b>Submitting LPCOpen issues:</b><br>
 * @ref LPCOPEN_COMMUNITY
 * @{
 */

/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/

/* Expected CRC values for test */
#define EXPECTED_CCIT_SUM       (0xFD2F)
#define EXPECTED_CRC16_SUM      (0x2799)
#define EXPECTED_CRC32_SUM      (0x100ECE8C)

/* Test block size in bytes */
#define BLOCK_SIZE 0x40

/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/

/*****************************************************************************
 * Private functions
 ****************************************************************************/

/* Populate block Data for tests */
static void initBlockData(uint8_t *p8)
{
	uint32_t i;

	for (i = 0; i < BLOCK_SIZE; i++) {
		p8[i] = i;
	}
}

/*****************************************************************************
 * Public functions
 ****************************************************************************/

/**
 * @brief	Main program body
 * @return	This function doesn't return
 */
int main(void) {
	uint32_t result;
	uint32_t BlockData[BLOCK_SIZE / 4];
	volatile int noExit = 1;/* For build warning only */

	/* Board Initialization */
	Board_Init();

	/* Chip Initialization */
	Chip_CRC_Init();

	Board_LED_Set(0, false);
	initBlockData((uint8_t *) BlockData);
	DEBUGSTR("CRC engine test\r\n");

	/* 8-bit CCIT */
	result = Chip_CRC_CRC8((const uint8_t *) BlockData, BLOCK_SIZE);
	DEBUGOUT("CCIT test results, expected 0x%x, got 0x%x\r\n",
			 EXPECTED_CCIT_SUM, result);
	if (result != EXPECTED_CCIT_SUM) {
		Board_LED_Set(0, true);
	}

	/* 16-bit CRC16 */
	result = Chip_CRC_CRC16((const uint16_t *) BlockData, BLOCK_SIZE / 2);
	DEBUGOUT("CRC16 test results, expected 0x%x, got 0x%x\r\n",
			 EXPECTED_CRC16_SUM, result);
	if (result != EXPECTED_CRC16_SUM) {
		Board_LED_Set(0, true);
	}

	/* 32-bit CRC16 */
	result = Chip_CRC_CRC32((const uint32_t *) BlockData, BLOCK_SIZE / 4);
	DEBUGOUT("CRC32 test results, expected 0x%x, got 0x%x\r\n",
			 EXPECTED_CRC32_SUM, result);
	if (result != EXPECTED_CRC32_SUM) {
		Board_LED_Set(0, true);
	}

	while (noExit) {}

	return 0;
}

/**
 * @}
 */
