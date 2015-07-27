/*
 * @brief SD/MMC example
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

#include <string.h>
#include "board.h"
#include "chip.h"
#include "ff.h"

/** @defgroup EXAMPLES_PERIPH_17XX40XX_SDC LPC17xx/40xx SDMMC example
 * @ingroup EXAMPLES_PERIPH_17XX40XX
 * <b>Example description</b><br>
 * This example describes how to use the SD/MMC interface with a SD card
 * and a FATFS filesystem.<br>
 *
 * To use the example, plug a SD card,  connect a serial cable to the board's
 * RS232/UART port and start a terminal program to monitor the port.  The
 * terminal program on the host PC should be setup for 115K8N1.<br>
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

#define debugstr(str)  DEBUGOUT(str)

/* buffer size (in byte) for R/W operations */
#define BUFFER_SIZE     4096

STATIC FATFS fatFS;	/* File system object */
STATIC FIL fileObj;	/* File object */
STATIC INT buffer[BUFFER_SIZE / 4];		/* Working buffer */
STATIC volatile int32_t sdcWaitExit = 0;
STATIC SDMMC_EVENT_T *event;
STATIC volatile Status  eventResult = ERROR;

/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/

/* SDMMC card info structure */
SDMMC_CARD_T sdCardInfo;
volatile uint32_t timerCntms = 0; /* Free running milli second timer */

/*****************************************************************************
 * Private functions
 ****************************************************************************/
/* Delay callback for timed SDIF/SDMMC functions */
STATIC void waitMs(uint32_t time)
{
	uint32_t init = timerCntms;

	/* In an RTOS, the thread would sleep allowing other threads to run.
	   For standalone operation, we just spin on a timer */
	while (timerCntms < init + time) {}
}

/**
 * @brief	Sets up the event driven wakeup
 * @param	pEvent : Event information
 * @return	Nothing
 */
STATIC void setupEvWakeup(void *pEvent)
{
#ifdef SDC_DMA_ENABLE
	/* Wait for IRQ - for an RTOS, you would pend on an event here with a IRQ based wakeup. */
	NVIC_ClearPendingIRQ(DMA_IRQn);
#endif
	event = (SDMMC_EVENT_T *)pEvent;
	sdcWaitExit = 0;
	eventResult = ERROR;
#ifdef SDC_DMA_ENABLE
	NVIC_EnableIRQ(DMA_IRQn);
#endif /*SDC_DMA_ENABLE*/
}

/**
 * @brief	A better wait callback for SDMMC driven by the IRQ flag
 * @return	0 on success, or failure condition (Nonzero)
 */
STATIC uint32_t waitEvIRQDriven(void)
{
	/* Wait for event, would be nice to have a timeout, but keep it  simple */
	while (sdcWaitExit == 0) {}
	if (eventResult) {
		return 0;
	}

	return 1;
}

/* Initialize the Timer at 1us */
STATIC void initAppTimer(void)
{
	/* Setup Systick to tick every 1 milliseconds */
	SysTick_Config(SystemCoreClock / 1000);
}

/* Initialize SD/MMC */
STATIC void initAppSDMMC()
{
	memset(&sdCardInfo, 0, sizeof(sdCardInfo));
	sdCardInfo.evsetup_cb = setupEvWakeup;
	sdCardInfo.waitfunc_cb = waitEvIRQDriven;
	sdCardInfo.msdelay_func = waitMs;

	Board_SDC_Init();

	Chip_SDC_Init(LPC_SDC);
}

/*****************************************************************************
 * Public functions
 ****************************************************************************/

/**
 * @brief	Error processing function: stop with dying message
 * @param	rc	: FatFs return value
 * @return	Nothing
 */
void die(FRESULT rc)
{
	DEBUGOUT("Failed with rc=%u.\r\n", rc);
	//	for (;; ) {}
}

/**
 * @brief	System tick interrupt handler
 * @return	Nothing
 */
void SysTick_Handler(void)
{
	timerCntms++;
}

#ifdef SDC_DMA_ENABLE
/**
 * @brief	GPDMA interrupt handler sub-routine
 * @return	Nothing
 */
void DMA_IRQHandler(void)
{
	eventResult = Chip_DMA_Interrupt(LPC_GPDMA, event->DmaChannel);
	sdcWaitExit = 1;
	NVIC_DisableIRQ(DMA_IRQn);
}
#endif /*SDC_DMA_ENABLE*/

/**
 * @brief	SDC interrupt handler sub-routine
 * @return	Nothing
 */
void SDIO_IRQHandler(void)
{
	int32_t Ret;
#ifdef SDC_DMA_ENABLE
	Ret = Chip_SDMMC_IRQHandler(LPC_SDC, NULL,0,NULL,0);
#else
	if(event->Index < event->Size) {
		if(event->Dir) { /* receive */
			Ret = Chip_SDMMC_IRQHandler(LPC_SDC, NULL,0,(uint8_t*)event->Buffer,&event->Index);
		}
		else {
			Ret = Chip_SDMMC_IRQHandler(LPC_SDC, (uint8_t*)event->Buffer,&event->Index,NULL,0);
		}
	}
	else {
		Ret = Chip_SDMMC_IRQHandler(LPC_SDC, NULL,0,NULL,0);
	}
#endif
	if(Ret < 0) {
		eventResult = ERROR;
		sdcWaitExit = 1;
	}
#ifndef SDC_DMA_ENABLE
	else if(Ret == 0) {
		eventResult = SUCCESS;
		sdcWaitExit = 1;
	}
#endif
}

/**
 * @brief	Main routine for SDMMC example
 * @return	Nothing
 */
int main(void)
{
	FRESULT rc;		/* Result code */
	DIR dir;		/* Directory object */
	FILINFO fno;	/* File information object */
	UINT bw, br, i;
	uint8_t *ptr;
	char debugBuf[64];

	Board_Init();

	initAppSDMMC();

	/* Initialize Repitetive Timer */
	initAppTimer();

	debugstr("\r\nHello NXP Semiconductors\r\nSD Card demo\r\n");

	/* Enable SD interrupt */
	NVIC_EnableIRQ(SDC_IRQn);

	f_mount(0, &fatFS);		/* Register volume work area (never fails) */

	debugstr("\r\nOpen an existing file (message.txt).\r\n");

	rc = f_open(&fileObj, "MESSAGE.TXT", FA_READ);
	if (rc) {
		die(rc);
	}
	else {
		for (;; ) {
			/* Read a chunk of file */
			rc = f_read(&fileObj, buffer, sizeof buffer, &br);
			if (rc || !br) {
				break;					/* Error or end of file */
			}
			ptr = (uint8_t *) buffer;
			for (i = 0; i < br; i++) {	/* Type the data */
				DEBUGOUT("%c", ptr[i]);
			}
		}
		if (rc) {
			die(rc);
		}

		debugstr("\r\nClose the file.\r\n");
		rc = f_close(&fileObj);
		if (rc) {
			die(rc);
		}
	}

	debugstr("\r\nCreate a new file (hello.txt).\r\n");
	rc = f_open(&fileObj, "HELLO.TXT", FA_WRITE | FA_CREATE_ALWAYS);
	if (rc) {
		die(rc);
	}
	else {

		debugstr("\r\nWrite a text data. (Hello world!)\r\n");

		rc = f_write(&fileObj, "Hello world!\r\n", 14, &bw);
		if (rc) {
			die(rc);
		}
		else {
			sprintf(debugBuf, "%u bytes written.\r\n", bw);
			debugstr(debugBuf);
		}
		debugstr("\r\nClose the file.\r\n");
		rc = f_close(&fileObj);
		if (rc) {
			die(rc);
		}
	}
	debugstr("\r\nOpen root directory.\r\n");
	rc = f_opendir(&dir, "");
	if (rc) {
		die(rc);
	}
	else {
		debugstr("\r\nDirectory listing...\r\n");
		for (;; ) {
			/* Read a directory item */
			rc = f_readdir(&dir, &fno);
			if (rc || !fno.fname[0]) {
				break;					/* Error or end of dir */
			}
			if (fno.fattrib & AM_DIR) {
				sprintf(debugBuf, "   <dir>  %s\r\n", fno.fname);
			}
			else {
				sprintf(debugBuf, "   %8lu  %s\r\n", fno.fsize, fno.fname);
			}
			debugstr(debugBuf);
		}
		if (rc) {
			die(rc);
		}
	}
	debugstr("\r\nTest completed.\r\n");
	for (;; ) {}
}

/**
 * @}
 */
