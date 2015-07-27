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

/** @defgroup EXAMPLES_DFUUTIL_18XX43XX_ANY LPC18xx/43xx IRAM programming algorithm for the DFU Utility
 * @ingroup EXAMPLES_DFUUTIL_18XX43XX
 * <b>Example description</b><br>
 * This programming algorithm allows upload and download of data to and from IRAM or
 * any peripheral registers. The entire 4GB memory range is accessible.<br>
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
#define PROGRAM_REGIONS 1

/* Size of DFU USB buffer in bytes, do not exceed 4K */
#define DFU_BUFF_PROG_SIZE 2048

/* Forward references */
int32_t progalgo_emiram_erase(uint32_t start, uint32_t size);

int32_t progalgo_emiram_erase_all(void);

int32_t progalgo_emiram_write(void *buff, uint32_t start, uint32_t size);

int32_t progalgo_emiram_read(void *buff, uint32_t start, uint32_t size);

/* Function table for exposed API functions */
static const PROGALGOS_T palgos = {
	progalgo_emiram_erase,
	progalgo_emiram_erase_all,
	progalgo_emiram_write,
	progalgo_emiram_read
};

/* 1 huge region with 4GB range, so we can stream large files */
static const DFUPROG_REGZONE_T pregions[PROGRAM_REGIONS] = {
	{0x00000000, 0xFFFFFFFC}
};

/* DFU programming region/API structure
   This structure puts them all together and is used by the DFU streamer */
static DFUPROG_REGION_T dfuregion = {
	PROGRAM_REGIONS,		/* Regions per device */
	DFU_BUFF_PROG_SIZE,	/* Size of buffer provided to DFU streamer */
	&palgos,			/* Pointer to programming algorithm function table */
	pregions,			/* Array of region addresses and sizes */
	DFUPROG_VALIDVAL,
	"iramregs_access"
};

/* Temporary work string for formatting */
static char tempSTR[64];

/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/

/*****************************************************************************
 * Private functions
 ****************************************************************************/

/* Erases a specific area of a device. This function can be optionally
   supported (by an empty function) if only the erase all function is used. */
static int32_t progalgo_emiram_erase(uint32_t start, uint32_t size)
{
	sprintf(tempSTR, "Erasing memory region: %p, size %d\n", (void *) start, size);
	usbDebug(tempSTR);
	memset((void *) start, 0, size);

	/* This function should block until the operation is complete. If the
	   operation fails, use the usbDebug() function to specify the error. */

	return size;
}

/* Erases all regions of all devices. This function can be optionally
   supported (with an empty function) in only the region erase is used. */
static int32_t progalgo_emiram_erase_all(void)
{
	usbDebug("Full device erase not supported for this algorithm, ignoring erase command\n");

	/* This function should block until the operation is complete. If the
	   operation fails, use the usbDebug() function to specify the error. */

	/* It's not supported, but don't generate an error */
	return 1;
}

/* Write the buffer to the device. Returns 0 if the region cannot
   be written (programming failure or region overlap) or the write
   size>0 if it passed. */
static int32_t progalgo_emiram_write(void *buff, uint32_t start, uint32_t size)
{
	sprintf(tempSTR, "WRITE @ 0x%p, %p bytes\n", (void *) start, (void *) size);
	usbDebug(tempSTR);
	memmove((void *) start, buff, size);

	/* This function should block until the operation is complete. If the
	   operation fails, use the usbDebug() function to specify the error. */

	return size;
}

/* Read data from the device. Returns 0 if the region cannot
   be read. */
static int32_t progalgo_emiram_read(void *buff, uint32_t start, uint32_t size)
{
	sprintf(tempSTR, "READ @ 0x%p, %p bytes\n", (void *) start, (void *) size);
	usbDebug(tempSTR);
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
	usbDebug("Memory mapped programming algorithm\n");

	/* Nothing to realliy initialize for this algorithm */
	return &dfuregion;
}

/**
 * @}
 */
