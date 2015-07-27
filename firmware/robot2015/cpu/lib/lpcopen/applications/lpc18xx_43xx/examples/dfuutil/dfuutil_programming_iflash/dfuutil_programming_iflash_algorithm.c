/*
 * @brief DFU Utility program for internal FLASH
 *        This programming algortihm allows reading or writing
 *        to the internal FLASH on 18xx/43xx devices.
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

/** @defgroup EXAMPLES_DFUUTIL_18XX43XX_IFLASH LPC18xx/43xx Internal FLASH programming algorithm for the DFU Utility
 * @ingroup EXAMPLES_DFUUTIL_18XX43XX
 * <b>Example description</b><br>
 * This programming algorithm allows upload and download of data to and from the internal
 * FLASH storage on LPC18xx and LPC43xx devices.<br>
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

/* LPC18xx/43xx IAP command defines */
#define IAP_INIT                                    49
#define IAP_PREP_SECS                           50
#define IAP_RAM_TO_FLASH                    51
#define IAP_ERASE_SECS                      52
#define IAP_BLANK_CHECK_SECS            53
#define IAP_READ_PART_ID                    54
#define IAP_READ_BOOT_CODE_VER      55
#define IAP_READ_DEV_SERIAL_NUM     58
#define IAP_COMPARE                             56
#define IAP_REINVOKE_ISP                    57
#define IAP_ERASE_PAGE                      59
#define IAP_SET_ACTIVE_FLASH_BANK   60

/* LPC18xx/43xx IAP command status defines (partial) */
#define IAP_COMMAND_SUCCESS             0x00
#define IAP_INVALID_COMMAND             0x01
#define IAP_SRC_ADDR_ERROR              0x02
#define IAP_DST_ADDR_ERROR              0x03
#define IAP_SRC_ADDR_NOT_MAPPED     0x04
#define IAP_DST_ADDR_NOT_MAPPED     0x05
#define IAP_COUNT_ERROR                     0x06
#define IAP_INVALID_SECTOR              0x - 7
#define IAP_SECTOR_NOT_BLANK            0x08
#define IAP_SECTOR_NOT_PREPARED     0x09
#define IAP_COMPARE_ERROR                   0x0A
#define IAP_BUSY                                    0x0B
#define IAP_PARAM_ERROR                     0x0C
#define IAP_ADDR_ERROR                      0x0D
#define IAP_ADDR_NOT_MAPPED             0x0E
#define IAP_CMD_LOCKED                      0x0F
#define IAP_INVALID_CODE                    0x10
#define IAP_INVALID_BAUD_RATE           0x11
#define IAP_INVALID_STOP_BIT            0x12
#define IAP_CRP_ENABLED                     0x13
#define IAP_INVALID_FLASH_UNIT      0x14
#define IAP_USER_CODE_CHECKSUM      0x15
#define IAP_ERROR_SETTING_ACTIVE_PART   0x16

/* Maximum size of FLASH */
#define FLASHMAXSIZE (512 * 1024)

/* Some IAP functions need to know the clock speed. If you don't
   know it, pick a -alow- speed. */
#define CPUCLOCKFRQINKHZ 90000

/* 18xx and 43xx sector information */
typedef struct {
	uint32_t sector_offset;
	uint32_t sector_size;
} SECTOR_INFO_T;
static const SECTOR_INFO_T sectorinfo[] = {
	{0x00000000, 0x00002000},	/* Offset 0x00002000, 8K size */
	{0x00002000, 0x00002000},	/* Offset 0x00004000, 8K size */
	{0x00004000, 0x00002000},	/* Offset 0x00006000, 8K size */
	{0x00006000, 0x00002000},	/* Offset 0x00008000, 8K size */
	{0x00008000, 0x00002000},	/* Offset 0x0000A000, 8K size */
	{0x0000A000, 0x00002000},	/* Offset 0x0000C000, 8K size */
	{0x0000C000, 0x00002000},	/* Offset 0x0000E000, 8K size */
	{0x0000E000, 0x00002000},	/* Offset 0x00010000, 8K size */
	{0x00010000, 0x00010000},	/* Offset 0x00020000, 64K size */
	{0x00020000, 0x00010000},	/* Offset 0x00030000, 64K size */
	{0x00030000, 0x00010000},	/* Offset 0x00040000, 64K size */
	{0x00040000, 0x00010000},	/* Offset 0x00050000, 64K size */
	{0x00050000, 0x00010000},	/* Offset 0x00060000, 64K size */
	{0x00060000, 0x00010000},	/* Offset 0x00070000, 64K size */
	{0x00070000, 0x00010000},	/* Offset 0x00000000, 64K size */
	{0xFFFFFFFF, 0xFFFFFFFF}	/* End of list */
};

/* FLASH page size */
#define PAGE_SIZE 512

/* IAP function and support structures */
typedef void (*IAP)(uint32_t *, uint32_t *);
#define IAP_LOCATION *((uint32_t *) 0x10400100);
static IAP iap_entry;
static uint32_t command[6], result[5];

/* Forward references */
int32_t progalgo_iflash_erase(uint32_t start, uint32_t size);

int32_t progalgo_iflash_erase_all(void);

int32_t progalgo_iflash_write(void *buff, uint32_t start, uint32_t size);

int32_t progalgo_iflash_read(void *buff, uint32_t start, uint32_t size);

/* DFU buffer size */
#define DFU_BUFF_PROG_SIZE PAGE_SIZE

/* Function table for exposed API functions */
static const PROGALGOS_T palgos = {
	progalgo_iflash_erase,
	progalgo_iflash_erase_all,
	progalgo_iflash_write,
	progalgo_iflash_read
};

/* Number of program regions */
#define PROGRAM_REGIONS 2

/* Regions and sizes for this device. The regions will be filled in after
   IAP init. */
static DFUPROG_REGZONE_T pregions[PROGRAM_REGIONS] = {
	{0x00000000, 0x00000000},
	{0x00000000, 0x00000000}
};

/* DFU programming region/API structure
   This structure puts them all together and is used by the DFU streamer */
static DFUPROG_REGION_T dfuregion = {
	0,					/* Up to 2 regions per device, set later */
	DFU_BUFF_PROG_SIZE,	/* Size of buffer provided to DFU streamer */
	&palgos,			/* Pointer to programming algorithm function table */
	pregions,			/* Array of region addresses and sizes */
	DFUPROG_VALIDVAL,
	"Internal FLASH"
};

/* Temporary work string for formatting */
static char tempSTR[128];

/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/

/*****************************************************************************
 * Private functions
 ****************************************************************************/

/* Find bank for an address */
static int progalgo_iflash_findbank(uint32_t addr)
{
	int bank;

	/* Bank 0 check first, then bank 1 */
	for (bank = 0; bank < dfuregion.num_regions; bank++) {
		if ((addr >= pregions[bank].region_addr) &&
			(addr < (pregions[bank].region_addr + pregions[bank].region_size))) {
			return bank;
		}
	}

	/* No matching bank, note this function returns -1 for an error, while
	   most return 0 for error. */
	return -1;
}

/* Verify a program address range is valid. It should not cross banks and
   should not excced the end of the banks address range. */
static int progalgo_iflash_progaddrvalid(uint32_t addr, uint32_t size)
{
	int bank, sg;

	/* Determine bank first */
	bank = progalgo_iflash_findbank(addr);
	if (bank < 0) {
		usbDebug("FLASH: Address does not map to bank\n");
		return 0;
	}

	/* The address must always be 512 byte aligned */
	if ((addr & (PAGE_SIZE - 1)) != 0) {
		usbDebug("FLASH: Address is not 512 byte aligned\n");
		return 0;
	}

	/* The size must be 512 byte aligned */
	sg = size / PAGE_SIZE;
	if (size != (PAGE_SIZE * sg)) {
		usbDebug("FLASH: Size must be a multiple of 512 bytes\n");
		return 0;
	}

	/* The range check can be done here, but both erase and programming
	   functions need to know the starting and ending sector for the address
	   range, so we'll let the sector find function handle it. */
	return (int) size;
}

/* For erase and write prepare operations, the starting and ending sector
   need to be known for an address range. This determines that sector
   range. Before calling this function, the address and size should have
   been validated with a call to progalgo_iflash_progaddrvalid(). */
static int progalgo_iflash_find_sectorrange(uint32_t addr, uint32_t size,
											int *bank, uint32_t *secstart, uint32_t *secend, int *aligned)
{
	int idx;
	uint32_t addrend, addrbase, regstart, regend;

	/* Bank number for address range */
	*bank = progalgo_iflash_findbank(addr);

	/* Base address for bank */
	addrbase = pregions[*bank].region_addr;
	addrend = addr + size - 1;

	/* The aligned flag is returned if the address range exactly maps to a
	   range of sectors boundaries. For erase operations, the aligned flag
	   must be set. */
	*aligned = 0;

	/* Find starting sector */
	idx = 0;
	*secstart = 0xFFFFFFFF;
	while ((*secstart == 0xFFFFFFFF) && (idx != -1)) {
		if (sectorinfo[idx].sector_offset == 0xFFFFFFFF) {
			idx = -1;
		}
		else {
			regstart = addrbase + sectorinfo[idx].sector_offset;
			regend = regstart + sectorinfo[idx].sector_size - 1;
			if ((addr >= regstart) && (addr <= regend)) {
				/* Falls in sector range */
				*secstart = (uint32_t) idx;

				/* Is starting address aligned? */
				if (addr == regstart) {
					*aligned = 1;
				}
				else {
					*aligned = 0;
				}
			}
			else {
				/* Check next range */
				idx++;
			}
		}
	}

	/* A starting sector was found? */
	if (idx == -1) {
		return 0;
	}

	/* Find the last sector for the address range */
	*secend = 0xFFFFFFFF;
	while ((*secend == 0xFFFFFFFF) && (idx != -1)) {
		if (sectorinfo[idx].sector_offset == 0xFFFFFFFF) {
			idx = -1;
		}
		else {
			regstart = addrbase + sectorinfo[idx].sector_offset;
			regend = regstart + sectorinfo[idx].sector_size - 1;
			if ((addrend >= regstart) && (addrend <= regend)) {
				/* Falls in sector range */
				*secend = (uint32_t) idx;

				/* Is ending address aligned? */
				if (addrend == regend) {
					*aligned = 1;
				}
				else {
					*aligned = 0;
				}
			}
			else {
				/* Check next range */
				idx++;
			}
		}
	}

	/* A ending sector was found? */
	if (idx == -1) {
		return 0;
	}

	return size;
}

/* Prepare a range of sectors for write/erase, returns a IAP status
   value of IAP_* */
static int progalgo_iflash_prepwrite(uint32_t bank, uint32_t secstart,
									 uint32_t secend)
{
	command[0] = IAP_PREP_SECS;
	command[1] = secstart;
	command[2] = secend;
	command[3] = bank;
	iap_entry(command, result);

	if (result[0] != IAP_COMMAND_SUCCESS) {
		sprintf(tempSTR, "ERASE: Error preparing sectors %d-%d (bank %d)\n",
				secstart, secend, bank);
		usbDebug(tempSTR);
	}

	return result[0];
}

/* Erase a range of sectors */
static int progalgo_iflash_erasesectors(uint32_t bank, uint32_t secstart,
										uint32_t secend)
{
	command[0] = IAP_ERASE_SECS;
	command[1] = secstart;
	command[2] = secend;
	command[3] = CPUCLOCKFRQINKHZ;
	command[4] = bank;
	iap_entry(command, result);

	if (result[0] != IAP_COMMAND_SUCCESS) {
		sprintf(tempSTR, "ERASE: Error erasing sectors %d-%d (bank %d)\n",
				secstart, secend, bank);
		usbDebug(tempSTR);
	}

	return result[0];
}

/* Erase a region of FLASH memory */
static int32_t progalgo_iflash_erase(uint32_t start, uint32_t size)
{
	int bank, aligned;
	uint32_t secstart, secend;

	sprintf(tempSTR, "ERASE: %p with size %p\n",
			(void *) start, (void *) size);
	usbDebug(tempSTR);

	/* Basic verification first of input parameters */
	if (progalgo_iflash_progaddrvalid(start, size) == 0) {
		usbDebug("ERASE: address/size validation failure\r\n");
		return 0;
	}

	/* Get sector and bank info for the operation */
	if (progalgo_iflash_find_sectorrange(start, size, &bank,
										 &secstart, &secend, &aligned) == 0) {
		usbDebug("ERASE: sector range lookup failure\r\n");
		return 0;
	}

	/* Must be aligned to a sector range! */
	if (aligned == 0) {
		usbDebug("ERASE: Address range must be sector aligned\r\n");
		return 0;
	}

	sprintf(tempSTR, "ERASE: Bank %d, Start sec %d, End sec %d\n",
			bank, secstart, secend);
	usbDebug(tempSTR);

	/* Prepare for write */
	if (progalgo_iflash_prepwrite(bank, secstart, secend) != IAP_COMMAND_SUCCESS) {
		return 0;
	}

	/* Erase sectors */
	if (progalgo_iflash_erasesectors(bank, secstart, secend) != IAP_COMMAND_SUCCESS) {
		return 0;
	}

	/* Verify they are erased */
	command[0] = IAP_BLANK_CHECK_SECS;
	command[1] = secstart;
	command[2] = secend;
	command[3] = bank;
	iap_entry(command, result);
	if (result[0] != IAP_COMMAND_SUCCESS) {
		usbDebug("ERASE: Error erasing sectors\r\n");
		return 0;
	}

	return size;
}

/* Erase the entire device */
static int32_t progalgo_iflash_erase_all(void)
{
	uint32_t bank, sz;

	/* Stop if there are no banks to erase */
	if (dfuregion.num_regions == 0) {
		return 0;
	}

	/* Erase 1 or 2 banks */
	for (bank = 0; bank < dfuregion.num_regions; bank++) {
		if (progalgo_iflash_erase(pregions[bank].region_addr,
								  pregions[bank].region_size) == 0) {
			return 0;
		}
	}

	sz = pregions[0].region_size;
	if (dfuregion.num_regions == 2) {
		sz += pregions[1].region_size;
	}

	return sz;
}

/* Write buffer to FLASH */
static int32_t progalgo_iflash_write(void *buff, uint32_t start, uint32_t size)
{
	int bank, aligned;
	uint32_t secstart, secend, offset, wsize = size;
	uint8_t *fbuff = (uint8_t *) buff;

	sprintf(tempSTR, "WRITE: %p with size %p\n", (void *) start, (void *) size);
	usbDebug(tempSTR);

	/* Verify size doesn't exceed buffer length. This check is not
	   really needed as the DFU streamer keeps the transfer at or
	   below the buffer size, but it's here for debug. */
	if (size > PAGE_SIZE) {
		usbDebug("WRITE: Program buffer too big\r\n");
		return 0;
	}

	/* If the passed size is less than the buffer size, then this is
	   the last fragment of code to program into FLASH. Since images to
	   program may not be on 512 byte boundaries, we'll fill up the
	   unused space with 0xFF to allow programming. */
	if (wsize < PAGE_SIZE) {
		for (; wsize < PAGE_SIZE; wsize++) {
			fbuff[wsize] = 0xFF;
		}

		sprintf(tempSTR, "WRITE: Last sector too small, padded %d bytes\n",
				(wsize - size));
		usbDebug(tempSTR);
	}

	/* Verify basic parameters */
	if (progalgo_iflash_progaddrvalid(start, wsize) == 0) {
		usbDebug("WRITE: Input address/size is not valid\r\n");
		return 0;
	}

	/* Get sector and bank info for the operation */
	if (progalgo_iflash_find_sectorrange(start, wsize, &bank,
										 &secstart, &secend, &aligned) == 0) {
		usbDebug("WRITE: sector range lookup failure\r\n");
		return 0;
	}

	offset = start - (pregions[bank].region_addr + sectorinfo[secstart].sector_offset);
	sprintf(tempSTR, "WRITE: programming: Bank %d, Start sec %d, End sec %d offset %d size %d\n",
			bank, secstart, secend, offset, wsize);
	usbDebug(tempSTR);

	/* Prepare for write */
	if (progalgo_iflash_prepwrite(bank, secstart, secend) != IAP_COMMAND_SUCCESS) {
		return 0;
	}

	/* Program the data */
	command[0] = IAP_RAM_TO_FLASH;
	command[1] = start;
	command[2] = (uint32_t) buff;
	command[3] = PAGE_SIZE;
	command[4] = CPUCLOCKFRQINKHZ;
	iap_entry(command, result);
	if (result[0] != IAP_COMMAND_SUCCESS) {
		usbDebug("WRITE: Error programming address range\r\n");
		return 0;
	}

	/* Verify */
	command[0] = IAP_COMPARE;
	command[1] = start;
	command[2] = (uint32_t) buff;
	command[3] = PAGE_SIZE;
	iap_entry(command, result);
	if (result[0] != IAP_COMMAND_SUCCESS) {
		usbDebug("WRITE: Verify error on program\r\n");
		return 0;
	}

	return size;
}

/* Read data from the device. Returns 0 if the region cannot
   be read. */
static int32_t progalgo_iflash_read(void *buff, uint32_t start, uint32_t size)
{
	sprintf(tempSTR, "READ @ 0x%p, %p bytes\n", (void *) start, (void *) size);
	usbDebug(tempSTR);

	/* SPI FLASH is memory mapped, so just copy the data to the buffer */
	memmove(buff, (void *) start, size);

	return size;
}

/* Attempt to compute FLASH size from ID */
static uint32_t progalgo_iflash_getsize(int bank, uint32_t part_id2)
{
	uint32_t flash_size;
	const int bank_shift[2] = {0, 4};
	const int bank_mask[2]  = {0x0F, 0xF0};

	flash_size = ((part_id2 & bank_mask[bank]) >> bank_shift[bank]) * 0x10000;
	if (FLASHMAXSIZE <= flash_size) {
		flash_size = 0;
	}
	else {
		flash_size = FLASHMAXSIZE - flash_size;
	}

	return flash_size;
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
	int idx;
	const uint32_t bankAddress[2] = {0x1A000000, 0x1B000000};
	uint32_t *iapTest32 = (uint32_t *) IAP_LOCATION;

	iap_entry = (IAP) IAP_LOCATION;
	dfuregion.num_regions = 0;

	/* Is IAP supported? */
	if (*iapTest32 == 0) {
		usbDebug("FLASH not supported on this device");
		return &dfuregion;
	}

	/* Initialize IAP */
	command[0] = IAP_INIT;
	iap_entry(command, result);
	if (result[0] == IAP_COMMAND_SUCCESS) {
		/* Get part ID */
		command[0] = IAP_READ_PART_ID;
		iap_entry(command, result);

		/* Setup regions and sizes based on part ID */
		for (idx = 0; idx < 2; idx++) {
			/* Up to 2 possible regions at 0x1A000000 and 0x1B000000 with varying sizes */
			pregions[idx].region_size = progalgo_iflash_getsize(idx, result[2]);
			if (pregions[idx].region_size > 0) {
				pregions[idx].region_addr = bankAddress[idx];
				dfuregion.num_regions++;
			}
			else {
				pregions[idx].region_addr = 0;
			}
		}
	}

	sprintf(tempSTR, "FLASH: banks = %d(0x%08x/0x%08x)\n", dfuregion.num_regions,
			result[1], result[2]);
	usbDebug(tempSTR);
	for (idx = 0; idx < dfuregion.num_regions; idx++) {
		sprintf(tempSTR, "FLASH bank %d: Start: %p, Size %p\n", idx,
				(void *) pregions[idx].region_addr, (void *) pregions[idx].region_size);
		usbDebug(tempSTR);
	}

	/* On FLASHless parts the number of regions will be 0 */
	return &dfuregion;
}

/**
 * @}
 */
