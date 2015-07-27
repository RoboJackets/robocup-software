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

/** @defgroup EXAMPLES_PERIPH_17XX40XX_NORFLASH LPC17xx/40xx External NOR Flash tests
 * @ingroup EXAMPLES_PERIPH_17XX40XX
 * <b>Example description</b><br>
 * The Flash example shows how to use external NOR Flash using EMC.<br>
 * The example allows you to place a small string in a sector of
 * the Flash and the string will remain across reset cycles.<br>
 *
 * To use the example, connect a serial cable to the board's RS232/UART port
 * and start a terminal program to monitor the port. The terminal program on
 * the host PC should be setup for 115K8N1.<br>
 *
 * <b>Special connection requirements</b><br>
 * Embedded Artists' LPC1788 Developer's Kit<br>
 * Embedded Artists' LPC4088 Developer's Kit<br>
 *		- JP1: ON
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

/*****************************************************************************
 * Private functions
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

/* Function for reading NOR FLASH - this is overkill, since NOR FLASH is
   is memory mapped and can be directly accessed. */
static void ReadFlash(uint32_t addr, uint16_t *buffer, uint32_t size)
{
	uint32_t i = 0;
	for (i = 0; i < BUFFER_SIZE; i += 2) {
		*buffer = lpc_norflash_read_word(addr + i);
		buffer++;
	}
}

static void WriteFlash(uint32_t addr, uint16_t *buffer, uint32_t size)
{
	lpc_norflash_write_buffer(addr, buffer, size);
}

/*****************************************************************************
 * Public functions
 ****************************************************************************/

/**
 * @brief	Main routine for example_norflash
 * @return	Nothing
 */
int main(void)
{
	uint32_t addr;
	uint16_t manufacturerID, deviceID;
	uint32_t sector_count, flash_size;
	uint32_t stSize;
	volatile int loop = 1;	/* For debug message only */

	Board_Init();

	/* Read flash information */
	lpc_norflash_get_size(&flash_size, &sector_count);
	lpc_norflash_get_id(&manufacturerID, &deviceID);
	DEBUGOUT(" Flash Information: \r\n");
	DEBUGOUT("      Manufacturer ID: 0x%04x\r\n", manufacturerID);
	DEBUGOUT("      Device ID: 0x%04x\r\n", deviceID);
	DEBUGOUT("      Size: %08dKB\r\n", flash_size / 1024);
	DEBUGOUT("      Sector Count: %08d\r\n", sector_count);

	/* Read data */
	addr = lpc_norflash_get_sector_offset(8);
	ReadFlash(addr, (uint16_t *) buffer, BUFFER_SIZE);

	/* Check and display string if it exists */
	ShowString((char *) buffer);

	/* Get a string to save */
	stSize = MakeString((uint8_t *) buffer);

	/* Erase flash */
	lpc_norflash_erase_sector(addr);
	/* Wait TSE (~25ms) */
	for (loop = 0; loop < 25 * 40000; loop++) {}

	/* Write header + size + data to page */
	DEBUGSTR("\r\nWrite to flash...\r\n");
	WriteFlash(addr, (uint16_t *) buffer, (4 + stSize));

	DEBUGSTR("Reading back string...\r\n");

	/* Read all data from flash */
	ReadFlash(addr, (uint16_t *) buffer, BUFFER_SIZE);

	/* Check and display string if it exists */
	ShowString((char *) buffer);

	/* Wait forever */
	while (loop) {}

	return 0;
}

/**
 * @}
 */
