/*
 * @brief Programming API used with DFU Utility programmer
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

#ifndef __DFUSEC_PROGRAMMING_API_H_
#define __DFUSEC_PROGRAMMING_API_H_

#include "board.h"

#ifdef __cplusplus
extern "C"
{
#endif

/** @defgroup EXAMPLES_DFUUTIL_18XX43XX_COMMON Common programming algorithm functions for the DFU Utility
 * @ingroup EXAMPLES_DFUUTIL_18XX43XX
 * <b>Example description</b><br>
 * This is the API file for the LPC18xx/43xx DFU Utility program. Flash algorithms
 * used with the DFU Utility must comply to the functions in this file.<br>
 *
 * <b>Submitting LPCOpen issues:</b><br>
 * @ref LPCOPEN_COMMUNITY
 * @{
 */

/**
 * Region address and size for a programming algorithm
 */
typedef struct {
	uint32_t region_addr;				/* Offset address */
	uint32_t region_size;				/* Size in bytes */
} DFUPROG_REGZONE_T;

/**
 * Pointer to programming algorithm region erase function
 */
typedef int32_t (*progalgo_flash_erase_region)(uint32_t, uint32_t);

/**
 * Pointer to programming algorithm erase all function
 */
typedef int32_t (*progalgo_flash_erase_all)(void);

/**
 * Pointer to programming algorithm write function
 * First parameter is the pointer to the buffer to write
 * Second parameter is the starting address to write (32-bit algined)
 * Third parameter is the number of bytes to write
 */
typedef int32_t (*progalgo_flash_write)(void *, uint32_t, uint32_t);

/**
 * Pointer to programming algorithm read function
 * First parameter is the pointer to the buffer to fill
 * Second parameter is the starting address to read (32-bit algined)
 * Third parameter is the number of bytes to read
 */
typedef int32_t (*progalgo_flash_read)(void *, uint32_t, uint32_t);

/**
 * Pointer to programming algorithm region erase function
 */
typedef struct {
	progalgo_flash_erase_region erase_region;
	progalgo_flash_erase_all    erase_all;
	progalgo_flash_write        write;
	progalgo_flash_read         read;
} PROGALGOS_T;

/**
 * Programming information structure defined for a algorithm
 */
typedef struct {
	/* Number of program regions on the device, it is recommended to
	   keep the number of regions less than 10 */
	int32_t num_regions;
	/* This must be a minimum of 64 bytes and a max of 4096 bytes. This must
	   be a factor (1x, 2x, 3x, etc.) of MAXP size */
	uint32_t buffer_size;
	const PROGALGOS_T *pprogalgos;
	const DFUPROG_REGZONE_T *pregions;	/* Address and region size array */
	uint32_t ver;	/* Versioning info */
	const char *algoname;	/* Unique name for the algorithm */
} DFUPROG_REGION_T;

/**
 * Initializes device programming capability
 */
DFUPROG_REGION_T *algo_flash_init(void);

/**
 * These are possible commands from the host machine
 */
typedef enum {
	DFU_HOSTCMD_SETDEBUG,   /* Enables/disables debug output */
	DFU_HOSTCMD_ERASE_ALL,		/* Erase the entire device */
	DFU_HOSTCMD_ERASE_REGION,	/* Erase a region defined with addr/size */
	DFU_HOSTCMD_PROGRAM,		/* Program a region defined with addr/size */
	DFU_HOSTCMD_READBACK,		/* Read a region defined with addr/size */
	DFU_HOSTCMD_RESET,			/* Reset the device/board */
	DFU_HOSTCMD_EXECUTE			/* Execute code at address addr */
} DFU_HOSTCMD_T;

/**
 * Host DFU download packet header. This is appended to a data packet when
 * programming a region.
 */
typedef struct {
	uint32_t hostCmd;		/* Host command */
	uint32_t addr;			/* Start of program/erase/read region, or execute address */
	uint32_t size;			/* Size of program/erase/read region */
	uint32_t magic;		/* Should be DFUPROG_VALIDVAL */
} DFU_FROMHOST_PACKETHDR_T;

/**
 * Magic value used to indicate DFU programming algorithm and DFU Utility
 * support. This is used to lock algorithm support to DFU Utility tool
 * version to prevent issues with non-compatible versions. The upper
 * 16 bits contain a magic number and the lower 16 bits contain the
 * version number in x.y format (1.10 = 0x010A).
 */
#define DFUPROG_VALIDVAL (0x18430000 | (0x010A))

/**
 * DFU operational status returned from programming algorithm, used
 * by host to monitor status of board
 */
typedef enum {
	DFU_OPSTS_IDLE,			/* Idle, can accept new host command */
	DFU_OPSTS_ERRER,		/* Erase error */
	DFU_OPSTS_PROGER,		/* Program error */
	DFU_OPSTS_READER,		/* Readback error */
	DFU_OPSTS_ERRUN,		/* Unknown error */
	DFU_OPSTS_READBUSY,		/* Device is busy reading a block of data */
	DFU_OPSTS_READTRIG,		/* Device data is ready to read */
	DFU_OPSTS_READREADY,	/* Block of data is ready */
	DFU_OPSTS_ERASE_ALL_ST,	/* Device is about to start full erase */
	DFU_OPSTS_ERASE_ST,		/* Device is about to start region erase */
	DFU_OPSTS_ERASE,		/* Device is currently erasing */
	DFU_OPSTS_PROG,			/* Device is currently programming a range */
	DFU_OPSTS_PROG_RSVD,	/* Reserved state, not used */
	DFU_OPSTS_PROG_STREAM,	/* Device is in buffer streaming mode */
	DFU_OPSTS_RESET,		/* Will shutdown and reset */
	DFU_OPSTS_EXEC,			/* Will shutdown USB and start execution */
	DFU_OPSTS_LOOP			/* Loop on error after DFU status check */
} DFU_OPSTS_T;

/**
 * When sending data to the host machine, a packet header is appended to
 * the data payload to indicate the state of the target and any debug or
 * error messages.
 */
typedef struct {
	uint32_t cmdResponse;	/* Command responding from host */
	uint32_t progStatus;	/* Current status of system */
	uint32_t strBytes;			/* Number of bytes in string field */
	uint32_t reserved;
} DFU_TOHOST_PACKETHDR_T;

/**
 * @brief	Queues a message for DFU status transfer
 * @param	str	: Message to queue
 * @return	Nothing
 */
void usbDebug(char *str);

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* __DFUSEC_PROGRAMMING_API_H_ */
