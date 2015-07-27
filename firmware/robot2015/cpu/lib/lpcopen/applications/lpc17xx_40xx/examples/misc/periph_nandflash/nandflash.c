/*
 * @brief	This example shows how to opertate on external flash operations.
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
#include "string.h"

/** @defgroup EXAMPLES_PERIPH_17XX40XX_NANDFLASH LPC17xx/40xx External NAND Flash tests
 * @ingroup EXAMPLES_PERIPH_17XX40XX
 * <b>Example description</b><br>
 * The Flash example shows how to use external NAND Flash using EMC.<br>
 * The example allows you to place a small string in a sector of
 * the Flash and the string will remain across reset cycles. No attempt at
 * ECC is made in this example (which is required for data reliability for
 * NAND FLASH).<br>
 *
 * To use the example, connect a serial cable to the board's RS232/UART port
 * and start a terminal program to monitor the port. The terminal program on
 * the host PC should be setup for 115K8N1.<br>
 *
 * <b>Special connection requirements</b><br>
 * JP2: 3-4 is ON
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

 /* The The desired index of block which is used to store string */
#define BLOCK_INDEX         1
/* The index of page which is used to store string */
#define PAGE_INDEX          0
/* BUFFER size */
#define BUFFER_SIZE         0x40

/* Tag for checking if a string already exists in Flash */
#define CHKTAG              "NxP"
#define CHKTAG_SIZE         3

/* ASCII ESC character code */
#define ESC_CHAR            27

/* Read/write buffer (32-bit aligned) */
uint32_t buffer[BUFFER_SIZE / sizeof(uint32_t)];

/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/

/* Show current string stored in UART */
static void ShowString(char *str) {
	int stSize;

	/* Is data tagged with check pattern? */
	if (strncmp(str, CHKTAG, CHKTAG_SIZE) == 0) {
		/* Get next byte, which is the string size in bytes */
		stSize = (uint32_t) str[3];
		if (stSize > 32) {
			stSize = 32;
		}

		/* Add terminator */
		str[4 + stSize] = '\0';

		/* Show string stored in Flash */
		DEBUGSTR("Stored string found in Flash\r\n");
		DEBUGSTR("-->");
		DEBUGSTR((char *) &str[4]);
		DEBUGSTR("<--\r\n");
	}
	else {
		DEBUGSTR("No string stored in the Flash\r\n");
	}
}

/* Get a string to save from the UART */
static uint32_t MakeString(uint8_t *str)
{
	int index, byte;
	char strOut[2];

	/* Get a string up to 32 bytes to write into Flash */
	DEBUGSTR("\r\nEnter a string to write into Flash\r\n");
	DEBUGSTR("Up to 32 bytes in length, press ESC to accept\r\n");

	/* Setup header */
	strncpy((char *) str, CHKTAG, CHKTAG_SIZE);

	/* Read until escape, but cap at 32 characters */
	index = 0;
	strOut[1] = '\0';
#if defined(DEBUG_ENABLE)
	byte = DEBUGIN();
	while ((index < 32) && (byte != ESC_CHAR)) {
		if (byte != EOF) {
			strOut[0] = str[4 + index] = (uint8_t) byte;
			DEBUGSTR(strOut);
			index++;
		}

		byte = DEBUGIN();
	}
#else
	/* Suppress warnings */
	(void) byte;
	(void) strOut;

	/* Debug input not enabled, so use a pre-setup string */
	strncpy((char *) &str[4], "12345678", 8);
	index = 8;
#endif

	str[3] = (uint8_t) index;

	return (uint32_t) index;
}

/*****************************************************************************
 * Private functions
 ****************************************************************************/

/* Wait for Nand Flash ready */
STATIC void waitForReady(void)
{
	if(Board_NANDFLash_GetReady()) {
		while(Board_NANDFLash_GetReady()){}
	}
	while(!Board_NANDFLash_GetReady()){}
}

/* Identify valid blocks */
bool checkBadBlock(uint32_t block)
{
	uint8_t valid;

	lpc_nandflash_read_start(block, 0,  K9F1G_SPARE_START_ADDR);
	waitForReady();
	lpc_nandflash_read_data(&valid, 1);
	return (bool) (valid != 0xFF);
}

/*****************************************************************************
 * Public functions
 ****************************************************************************/

/**
 * @brief	Main routine for example_nandflash
 * @return	Nothing
 */
int main(void)
{
	K9F1G_ID_T nandId;
	const lpc_nandflash_size_t *flashInfo;
	uint32_t stSize, useBlock = BLOCK_INDEX;
	volatile int loop = 1, idx;	/* For debug message only */

	Board_Init();
	Board_NANDFLash_Init();
	lpc_nandflash_init();

	/* Read flash information */
	flashInfo = lpc_nandflash_get_size();
	lpc_nandflash_get_id((uint8_t*)&nandId);
	DEBUGOUT(" Flash Information: \r\n");
	DEBUGOUT("      Manufacturer ID: 0x%02x\r\n", nandId.MarkerCode);
	DEBUGOUT("      Device ID: 0x%02x\r\n", nandId.DeviceCode);
	DEBUGOUT("      Page Size: %dB\r\n", flashInfo->page_size);
	DEBUGOUT("      Spare Size: %dB\r\n", flashInfo->spare_size);
	DEBUGOUT("      Block Size: %dKB\r\n", flashInfo->pages_per_block*flashInfo->page_size/1024);
	DEBUGOUT("      Block count: %d\r\n", flashInfo->block_cnt);

	/* Show bad block list */
	DEBUGOUT("Checking bad blocks...\r\n");
	for (idx = 0; idx < flashInfo->block_cnt; idx++) {
		if (checkBadBlock(idx)) {
			DEBUGOUT("      Bad block at %d\r\n", idx);
			if (useBlock == idx) {
				/* Skip to next block for the example if this one is bad */
				useBlock++;
			}
		}
	}

	/* Read data */
	lpc_nandflash_read_start(useBlock, PAGE_INDEX, 0);
	waitForReady();
	lpc_nandflash_read_data((uint8_t *) buffer, BUFFER_SIZE);

	/* Check and display string if it exists */
	ShowString((char *) buffer);

	/* Get a string to save */
	stSize = MakeString((uint8_t *) buffer);

	/* Erase flash */
	lpc_nandflash_erase_block(useBlock);
	waitForReady();

	/* Check the result of erasing */
	if(lpc_nandflash_read_status() & NANDFLASH_STATUS_BLOCK_ERASE_FAIL) {
		DEBUGSTR("Erase failed!!!\r\n");
		while(1){}
	}

	/* Write header + size + data to page */
	DEBUGSTR("\r\nWrite to flash...\r\n");
	lpc_nandflash_write_page(useBlock, PAGE_INDEX,(uint8_t *) buffer, (4 + stSize));
	waitForReady();
	
	/* Check the result of writting */
	if(lpc_nandflash_read_status() & NANDFLASH_STATUS_PAGE_PROG_FAIL) {
		DEBUGSTR("Writing failed!!!\r\n");
		while(1){}
	}

	DEBUGSTR("Reading back string...\r\n");

	/* Read all data from flash */
	lpc_nandflash_read_start(useBlock, PAGE_INDEX, 0);
	waitForReady();
	lpc_nandflash_read_data((uint8_t *) buffer, BUFFER_SIZE);

	/* Check and display string if it exists */
	ShowString((char *) buffer);

	/* Wait forever */
	while (loop) {}

	return 0;
}

/**
 * @}
 */
