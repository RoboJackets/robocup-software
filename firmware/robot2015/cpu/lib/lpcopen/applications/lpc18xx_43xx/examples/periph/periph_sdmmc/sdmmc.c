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
#include "rtc.h"
#include "ff.h"

/** @defgroup EXAMPLES_PERIPH_18XX43XX_SDMMC LPC18xx/43xx SDMMC example
 * @ingroup EXAMPLES_PERIPH_18XX43XX
 * <b>Example description</b><br>
 * This example describes how to use the SD/MMC interface with a SD card
 * and a FATFS filesystem.<br>
 *
 * To use the example, plug a SD card (Hitex A4 board) or microSD card (NGX or Keil
 * boards) and connect a serial cable to the board's RS232/UART port start a terminal 
 * program to monitor the port.  The terminal program on the host PC should be setup 
 * for 115K8N1. <br>
 *
 * <b>Special connection requirements</b><br>
 *  - Hitex LPC1850EVA-A4-2 and LPC4350EVA-A4-2 boards<br>
 *     - Close pin 1-2 of JP9<br>
 *     - Open all SV3, SV6<br>
 *     - Close all SV12<br>
 *     - Uart1 terminal port: SV11-p2(U1_RXD) connect to SV1-p7, SV11-p4 (U1_TXD) connect to SV1-p5<br>
 *  - Keil MCB1857 and MCB4357 boards (debug output terminal on Uart3)<br>
 *  - NGX LPC1830 LPC4330 (no debug output terminal)<br>
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
#ifdef BOARD_HITEX_EVA_18504350
#define debugstr(str)  board_uart_out_string(str)
#else
#define debugstr(str)  DEBUGSTR(str)
#endif

/* buffer size (in byte) for R/W operations */
#define BUFFER_SIZE     4096

static FATFS Fatfs;	/* File system object */
static FIL Fil;	/* File object */
static uint32_t Buff[BUFFER_SIZE/sizeof(uint32_t)];

static volatile UINT Timer = 0;		/* Performance timer (1kHz increment) */
static volatile int32_t sdio_wait_exit = 0;

/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/

/* SDMMC card info structure */
mci_card_struct sdcardinfo;

/*****************************************************************************
 * Private functions
 ****************************************************************************/

/* Delay callback for timed SDIF/SDMMC functions */
static void sdmmc_waitms(uint32_t time)
{
	/* In an RTOS, the thread would sleep allowing other threads to run. 
	   For standalone operation, we just spin on RI timer */
	int32_t curr = (int32_t) Chip_RIT_GetCounter(LPC_RITIMER);
	int32_t final = curr + ((SystemCoreClock / 1000) * time);
	
	if (final == curr) return;

	if ((final < 0) && (curr > 0)) {
		while (Chip_RIT_GetCounter(LPC_RITIMER) < (uint32_t) final) {}
	}
	else {
		while ((int32_t) Chip_RIT_GetCounter(LPC_RITIMER) < final) {}
	}

	return;
}

/**
 * @brief	Sets up the SD event driven wakeup
 * @param	bits : Status bits to poll for command completion
 * @return	Nothing
 */
static void sdmmc_setup_wakeup(void *bits)
{
	uint32_t bit_mask = *((uint32_t *)bits);
	/* Wait for IRQ - for an RTOS, you would pend on an event here with a IRQ based wakeup. */
	NVIC_ClearPendingIRQ(SDIO_IRQn);
	sdio_wait_exit = 0;
	Chip_SDIF_SetIntMask(LPC_SDMMC, bit_mask);
	NVIC_EnableIRQ(SDIO_IRQn);
}

/**
 * @brief	A better wait callback for SDMMC driven by the IRQ flag
 * @return	0 on success, or failure condition (-1)
 */
static uint32_t sdmmc_irq_driven_wait(void)
{
	uint32_t status;

	/* Wait for event, would be nice to have a timeout, but keep it  simple */
	while (sdio_wait_exit == 0) {}

	/* Get status and clear interrupts */
	status = Chip_SDIF_GetIntStatus(LPC_SDMMC);

	return status;
}

/* Initialize SD/MMC */
static void App_SDMMC_Init()
{
	memset(&sdcardinfo, 0, sizeof(sdcardinfo));
	sdcardinfo.card_info.evsetup_cb = sdmmc_setup_wakeup;
	sdcardinfo.card_info.waitfunc_cb = sdmmc_irq_driven_wait;
	sdcardinfo.card_info.msdelay_func = sdmmc_waitms;

	/*  SD/MMC initialization */
	Board_SDMMC_Init();

	/* The SDIO driver needs to know the SDIO clock rate */
	Chip_SDIF_Init(LPC_SDMMC);
}

#ifdef BOARD_HITEX_EVA_18504350
/* Initialize the UART for debugging */
static void board_uart_debug_init(void)
{
	/*  Hitex EVA A4 is sharing Uart port 0 and SDIO pin so it is needed to use Uart port 1 */
	Board_UART_Init(LPC_UART1);
	Chip_UART_SetBaud(LPC_UART1, 115200);
	Chip_UART_ConfigData(LPC_UART1, UART_DATABIT_8, UART_PARITY_NONE, UART_STOPBIT_1);	/*  default 8-N-1 */

	/*  Enable UART Transmit */
	Chip_UART_TxCmd(LPC_UART1, ENABLE);
}

/* Sends a character on the UART */
static void board_uart_out_ch(char ch)
{
	while (Chip_UART_SendByte(LPC_UART1, (uint8_t) ch) == ERROR) {}
}

/* Sends a string on the UART */
static void board_uart_out_string(char *str)
{
	while (*str != '\0') {
		board_uart_out_ch(*str++);
	}
}
#endif

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
	DEBUGOUT("Failed with rc=%u.\n", rc);
/*	for (;; ) {} */
}

/**
 * @brief	SDIO controller interrupt handler
 * @return	Nothing
 */
void SDIO_IRQHandler(void)
{
	/* All SD based register handling is done in the callback
	   function. The SDIO interrupt is not enabled as part of this
	   driver and needs to be enabled/disabled in the callbacks or
	   application as needed. This is to allow flexibility with IRQ
	   handling for applicaitons and RTOSes. */
	/* Set wait exit flag to tell wait function we are ready. In an RTOS,
	   this would trigger wakeup of a thread waiting for the IRQ. */
	NVIC_DisableIRQ(SDIO_IRQn);
	sdio_wait_exit = 1;
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
	char debugBuf[64];
	char *cbuf = (char *) Buff;

	Board_Init();
#ifdef BOARD_HITEX_EVA_18504350
	board_uart_debug_init();
#endif

	App_SDMMC_Init();
	
	/* There is no need to initialize RTC here it is called by fs init
	   but it takes long time hence doing it before calling f_open */
	debugstr("Initializing RTC (might take few seconds)...");
	rtc_initialize();
	debugstr("Done\r\n");

	debugstr("Hello NXP Semiconductors\r\n\r\n***SDCARD demo***\r\n");

	NVIC_DisableIRQ(SDIO_IRQn);
	/* Enable SD/MMC Interrupt */
	NVIC_EnableIRQ(SDIO_IRQn);

	f_mount(0, &Fatfs);		/* Register volume work area (never fails) */
	
	debugstr("Opening MESSAGE.TXT from SD Card...");
	rc = f_open(&Fil, "MESSAGE.TXT", FA_READ);
	if (rc) {
		debugstr("Failed.\r\n");
		die(rc);
	}
	debugstr("Done.\r\n");

	debugstr("Printing contents of MESSAGE.TXT...\r\n");
	for (;; ) {
		/* Read a chunk of file */
		rc = f_read(&Fil, (void *)cbuf, BUFFER_SIZE, &br);
		if (rc || !br) {
			break;					/* Error or end of file */
		}
		for (i = 0; i < br; i++)	/* Type the data */
#ifdef BOARD_HITEX_EVA_18504350
		{board_uart_out_ch((char) cbuf[i]); }
#else
		{DEBUGOUT("%c", cbuf[i]); }
#endif
	}
	if (rc) {
		die(rc);
	}

	debugstr("Close the file.\r\n");
	rc = f_close(&Fil);
	if (rc) {
		die(rc);
	}

	debugstr("Create a new file (hello.txt).\r\n");
	rc = f_open(&Fil, "HELLO.TXT", FA_WRITE | FA_CREATE_ALWAYS);
	if (rc) {
		die(rc);
	}

	debugstr("Write text data \"Hello world!\" to HELLO.TXT\r\n");

	rc = f_write(&Fil, "Hello world!\r\n", 14, &bw);
	if (rc) {
		die(rc);
	}

	sprintf(debugBuf, "%u bytes written.\r\n", bw);
	debugstr(debugBuf);

	debugstr("Close the file.\r\n");
	rc = f_close(&Fil);
	if (rc) {
		die(rc);
	}

	debugstr("Open root directory.\r\n");
	rc = f_opendir(&dir, "");
	if (rc) {
		die(rc);
	}

	debugstr("Directory listing...\r\n");
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

	debugstr("Test completed.\r\n");
	for (;; ) {}
}

/**
 * @}
 */
