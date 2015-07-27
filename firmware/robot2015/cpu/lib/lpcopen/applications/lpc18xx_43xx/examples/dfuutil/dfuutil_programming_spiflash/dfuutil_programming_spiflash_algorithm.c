/*
 * @brief DFU Utility program for IRAM/peripheral addresses
 *        This programming algortihm allows reading or writing
 *        any address in the device.
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
#include "dfuutil_programming_api.h"
#include "stdio.h"
#include "string.h"

/* Use the SPIFI library instead of ROM PTR table */
#define USE_SPIFI_LIB
#include "spifi_rom_api.h"

/** @defgroup EXAMPLES_DFUUTIL_18XX43XX_SIIFLASH SPI FLASH programming algorithm for the DFU Utility
 * @ingroup EXAMPLES_DFUUTIL_18XX43XX
 * <b>Example description</b><br>
 * This programming algorithm allows upload and download of data to and from SPI FLASH
 * storage on LPC18xx and LPC43xx devices.<br>
 *
 * More information on using the DFU Utility can be found at the following links:<br>
 * <a href="http://www.lpcware.com/content/project/dfu-download-programming-utility-and-security-lpcdfusec-tool">DFU Utility information</a><br>
 * <a href="http://www.lpcware.com/content/project/dfu-download-programming-utility-and-security-lpcdfusec-tool/dfusec-dfu-production-p">DFU Utility production programming API</a><br>
 *
 * <b>Special connection requirements</b><br>
 * There are no special connection requirements for this example.<br>
 *
 * <b>Build procedures:</b><br>
 * @ref LPCOPEN_18XX43XX_BUILDPROCS_KEIL<br>
 *
 * <b>Supported boards and board setup:</b><br>
 * Although this example includes a build target for only the Keil 1857 board,
 * it should run on all 18xx or 43xx boards.<br>
 * @ref LPCOPEN_18XX_BOARD_KEIL1857<br>
 *
 * <b>Submitting LPCOpen issues:</b><br>
 * @ref LPCOPEN_COMMUNITY
 * @{
 */

/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/

/* Number of program regions */
#define PROGRAM_REGIONS 2

/* Size of DFU USB buffer in bytes, do not exceed 4K */
#define DFU_BUFF_PROG_SIZE 2048

/* Forward references */
int32_t progalgo_spiflash_erase(uint32_t start, uint32_t size);

int32_t progalgo_spiflash_erase_all(void);

int32_t progalgo_spiflash_write(void *buff, uint32_t start, uint32_t size);

int32_t progalgo_spiflash_read(void *buff, uint32_t start, uint32_t size);

/* Function table for exposed API functions */
static const PROGALGOS_T palgos = {
	progalgo_spiflash_erase,
	progalgo_spiflash_erase_all,
	progalgo_spiflash_write,
	progalgo_spiflash_read
};

/* Regions and sizes for this device. The regions will be filled in after
   IAP init. */
static DFUPROG_REGZONE_T pregions[PROGRAM_REGIONS] = {
	{0x00000000, 0x00000000},
	{0x00000000, 0x00000000}
};

/* DFU programming region/API structure
   This structure puts them all together and is used by the DFU streamer */
static DFUPROG_REGION_T dfuregion = {
	PROGRAM_REGIONS,		/* Regions per device */
	DFU_BUFF_PROG_SIZE,	/* Size of buffer provided to DFU streamer */
	&palgos,			/* Pointer to programming algorithm function table */
	pregions,			/* Array of region addresses and sizes */
	DFUPROG_VALIDVAL,
	"SPIFLASH"
};

/* Needed SPIFI library supoprt objects */
static SPIFIobj spiobj;
static SPIFIopers spiopers;

/* SPIFI FLASH good initialization flag */
static int SpiGood;

/* Temporary work string for formatting */
static char tempSTR[128];

/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/

/*****************************************************************************
 * Private functions
 ****************************************************************************/

/* Alternate region */
static uint32_t progalgo_spiflash_chk_alt(uint32_t addr)
{
	if ((addr & 0xFF000000) == pregions[1].region_addr) {
		addr &= ~0xFF000000;
		addr |= pregions[0].region_addr;
	}

	return addr;
}

/* Verify a program address range is valid */
static int progalgo_spiflash_progaddrvalid(uint32_t addr, uint32_t size)
{
	if (!SpiGood) {
		return 0;
	}

	/* Make sure operation is 32-bit algined */
	if (addr & 0x3) {
		return 0;
	}

	addr = progalgo_spiflash_chk_alt(addr);
	if ((addr >= dfuregion.pregions[0].region_addr) &&
		((addr + size) <= (dfuregion.pregions[0].region_addr + dfuregion.pregions[0].region_size))) {
		return size;
	}

	return 0;
}

/* Region erase not supported on SPI FLASH */
static int32_t progalgo_spiflash_erase(uint32_t start, uint32_t size)
{
	if (progalgo_spiflash_progaddrvalid(start, size) == 0) {
		return 0;
	}

	sprintf(tempSTR, "ERASE region: Start %p, size %p\n", (void *) start, (void *) size);
	usbDebug(tempSTR);

	start = progalgo_spiflash_chk_alt(start);

	/* Erase just selected region */
	spiopers.dest = (char *) start;
	spiopers.length = size;
	spiopers.scratch = NULL;
	spiopers.options = S_VERIFY_ERASE;

	if (spifi_erase(&spiobj, &spiopers)) {
		return 0;
	}

	return size;
}

/* Erse entire FPI FLASh device */
static int32_t progalgo_spiflash_erase_all(void)
{
	if (!SpiGood) {
		return 0;
	}

	usbDebug("ERASE: entire device\r\n");

	spiopers.dest = (char *) (spiobj.base);
	spiopers.length = spiobj.memSize;
	spiopers.scratch = NULL;
	spiopers.options = S_VERIFY_ERASE;

	if (spifi_erase(&spiobj, &spiopers)) {
		return 0;
	}

	return spiobj.memSize;
}

/* Write the buffer to the device. Returns 0 if the region cannot
   be written (programming failure or region overlap) or the write
   size>0 if it passed. */
static int32_t progalgo_spiflash_write(void *buff, uint32_t start, uint32_t size)
{

	uint8_t *p8s = (uint8_t *) start, *p8d = (uint8_t *) buff;
	uint32_t vsz = size;
	int tmp;

	/* Check address range and alternate range */
	start = progalgo_spiflash_chk_alt(start);
	if (progalgo_spiflash_progaddrvalid(start, size) == 0) {
		return 0;
	}

	/* Setup SPI FLASH operation via the SPI FLASH driver */
	spiopers.dest = (char *) start;
	spiopers.length = size;
	spiopers.scratch = (void *) NULL;
	spiopers.protect = 0;
	spiopers.options = S_CALLER_ERASE;

	/* Setup SPI FLASH operation via the SPI FLASH driver */
	sprintf(tempSTR, "PROG @ 0x%p, %p bytes\n", (void *) start, (void *) size);
	usbDebug(tempSTR);

	tmp = spifi_program(&spiobj, (char *) p8d, &spiopers);
	if (tmp != 0) {
		sprintf(tempSTR, "PROG fail: status %x at %p, size %p\n", tmp, (void *) start, (void *) size);
		usbDebug(tempSTR);
		return 0;
	}

	/* Verify */
	while (vsz > 0) {
		if (*p8s != *p8d) {
			sprintf(tempSTR, "PROG verify fail: address %p, is: %x, should be: %x\n", p8s, *p8s, *p8d);
			usbDebug(tempSTR);
			return 0;
		}
		p8d++;
		p8s++;
		vsz--;
	}

	return size;
}

/* Read data from the device. Returns 0 if the region cannot
   be read. */
static int32_t progalgo_spiflash_read(void *buff, uint32_t start, uint32_t size)
{
	sprintf(tempSTR, "READ @ 0x%p, %p bytes\n", (void *) start, (void *) size);
	usbDebug(tempSTR);

	/* SPI FLASH is memory mapped, so just copy the data to the buffer */
	memmove(buff, (void *) start, size);

	return size;
}

/*****************************************************************************
 * Public functions
 ****************************************************************************/

/**
 * @brief	Initializes device programming capability
 * @return	A pointer to the programming info structure
 * Initializes device programming capability. Returns a pointer to the
 * programming buffer, the programming buffer size, and a pointer to the
 * DFU programming region/API structure used by the DFU streamer.
 */
DFUPROG_REGION_T *algo_flash_init(void)
{
	SpiGood = 0;

	/* Initialize SPI FLASH */
	if (spifi_init(&spiobj, 3, S_RCVCLK | S_FULLCLK, 12)) {
		usbDebug("SPIFLASH check: initialization failed\r\n");
		return &dfuregion;	/* Failed */
	}

	SpiGood = 1;

	/* Set first region */
	pregions[0].region_addr = (uint32_t) spiobj.base;
	pregions[0].region_size = pregions[1].region_size = (uint32_t) spiobj.memSize;

	/* Allow alternate address for SPI FLASH programming */
	if (pregions[0].region_addr == 0x80000000) {
		pregions[1].region_addr = 0x14000000;
	}
	else {
		pregions[1].region_addr = 0x80000000;
	}

	sprintf(tempSTR, "SPIFLASH: device base %p with size %p bytes\r\n",
			(void *) spiobj.base, (void *) spiobj.memSize);
	usbDebug(tempSTR);
	sprintf(tempSTR, "SPIFLASH: Alternate device base %p with size %p bytes\r\n",
			(void *) pregions[1].region_addr, (void *) pregions[1].region_size);
	usbDebug(tempSTR);

	return &dfuregion;
}

/**
 * @}
 */
