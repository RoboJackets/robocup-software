/*
 * @brief	lwIP Filesystem implementation module
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

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "lwip/mem.h"
#include "lwip/memp.h"
#include "board.h"
#include "ff.h"
#include "fs_mem.h"
#include "lwip_fs.h"
#include "httpd_structs.h"

/**
 * @ingroup EXAMPLE_LWIP_WEBSERVER_18XX43XX_FS
 * @{
 */

/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/

/* Default html file */
const static char http_index_html[] =
	"<html><head><title>Congrats!</title></head>"
	"<body><h1>Welcome to our lwIP HTTP server!</h1>"
	"<p>This is a small test page, served by httpd of "
	"lwip.</p></body></html>";

static FATFS Fatfs;	/* File system object */

/* Internal File descriptor structure */
struct file_ds {
	uint8_t scratch[SECTOR_SZ];
	FIL fi;
	struct fs_file fs;
	int fi_valid;
};
static volatile int32_t sdio_wait_exit = 0;

/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/
/* SDMMC card info structure */
mci_card_struct sdcardinfo;

/*****************************************************************************
 * Private functions
 ****************************************************************************/

#ifdef OS_FREE_RTOS
#include "FreeRTOS.h"
#include "semphr.h"
static xSemaphoreHandle open_lock;
/* FreeRTOS mutex lock */
static int mutex_lock(xSemaphoreHandle *mx)
{
	if (xSemaphoreTake(*mx, 500) != pdTRUE) {
		return 1;
	}
	return 0;
}

/* FreeRTOS mutex unlock */
static void mutex_unlock(xSemaphoreHandle *mx)
{
	xSemaphoreGive(*mx);
}

/* FreeRTOS mutex init */
static int mutex_init(void)
{
	open_lock = xSemaphoreCreateMutex();
	return open_lock == NULL;
}

#elif defined(OS_UCOS_III)

#include "os.h"
#include "cpu.h"
#include "os_cfg_app.h"

static OS_MUTEX *open_lock;

/* mutex lock function for ucos-iii */
static int mutex_lock(OS_MUTEX **mx)
{
	CPU_TS ts;
	OS_ERR os_err;
	OSMutexPend(*mx, 500, OS_OPT_PEND_BLOCKING, &ts, &os_err);
	if (os_err != OS_ERR_NONE) {
		return 1;
	}

	return 0;
}

/* Mutex unlock function for ucos-iii */
static void mutex_unlock(OS_MUTEX **mx)
{
	OS_ERR os_err;
	OSMutexPost(*mx, OS_OPT_POST_NONE, &os_err);
}

/* Mutex init function for ucos-iii */
static int mutex_init(void)
{
	static OS_MUTEX fsmutex;
	OS_ERR os_err;
	OSMutexCreate(&fsmutex, "DOS_FS_Mutex", &os_err);
	if (os_err == OS_ERR_NONE) {
		open_lock = &fsmutex;
		return 0;
	}
	else {
		return 1;
	}
}

#else
static int open_lock;
/* Mutex lock function for standalone */
static int mutex_lock(int *mx)
{
	return 0;
}

/* Mutex init for standalone */
static int mutex_init(void)
{
	open_lock = 1;
	return 0;
}

/* Mutex lock for standalone */
static void mutex_unlock(int *mx)
{}

#endif

/**
 * Generate the relevant HTTP headers for the given filename and write
 * them into the supplied buffer.
 */
static int
get_http_headers(const char *fName, char *buff)
{
	unsigned int iLoop;
	const char *pszExt = NULL;
	const char *hdrs[4];

	/* Ensure that we initialize the loop counter. */
	iLoop = 0;

	/* In all cases, the second header we send is the server identification
	   so set it here. */
	hdrs[1] = g_psHTTPHeaderStrings[HTTP_HDR_SERVER];

	/* Is this a normal file or the special case we use to send back the
	   default "404: Page not found" response? */
	if (( fName == NULL) || ( *fName == 0) ) {
		hdrs[0] = g_psHTTPHeaderStrings[HTTP_HDR_NOT_FOUND];
		hdrs[2] = g_psHTTPHeaderStrings[DEFAULT_404_HTML];
		goto end_fn;
	}
	/* We are dealing with a particular filename. Look for one other
	    special case.  We assume that any filename with "404" in it must be
	    indicative of a 404 server error whereas all other files require
	    the 200 OK header. */
	if (strstr(fName, "404")) {
		iLoop = HTTP_HDR_NOT_FOUND;
	}
	else if (strstr(fName, "400")) {
		iLoop = HTTP_HDR_BAD_REQUEST;
	}
	else if (strstr(fName, "501")) {
		iLoop = HTTP_HDR_NOT_IMPL;
	}
	else {
		iLoop = HTTP_HDR_OK;
	}
	hdrs[0] = g_psHTTPHeaderStrings[iLoop];

	/* Get a pointer to the file extension.  We find this by looking for the
	     last occurrence of "." in the filename passed. */
	pszExt = strrchr(fName, '.');

	/* Does the FileName passed have any file extension?  If not, we assume it
	     is a special-case URL used for control state notification and we do
	     not send any HTTP headers with the response. */
	if (pszExt == NULL) {
		return 0;
	}

	pszExt++;
	/* Now determine the content type and add the relevant header for that. */
	for (iLoop = 0; (iLoop < NUM_HTTP_HEADERS); iLoop++)
		/* Have we found a matching extension? */
		if (!strcmp(g_psHTTPHeaders[iLoop].extension, pszExt)) {
			hdrs[2] =
				g_psHTTPHeaderStrings[g_psHTTPHeaders[iLoop].headerIndex];
			break;
		}

	/* Did we find a matching extension? */
	if (iLoop == NUM_HTTP_HEADERS) {
		/* No - use the default, plain text file type. */
		hdrs[2] = g_psHTTPHeaderStrings[HTTP_HDR_DEFAULT_TYPE];
	}

end_fn:
	iLoop = strlen(hdrs[0]);
	strcpy(buff, hdrs[0]);
	strcat(buff, hdrs[1]);
	strcat(buff, hdrs[2]);
	return strlen(buff);
}
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

/*****************************************************************************
 * Public functions
 ****************************************************************************/

/* Read http header information into a string */
int GetHTTP_Header(const char *fName, char *buff)
{
	return get_http_headers(fName, buff);
}

/* Initialize the file system */
int fs_init(void)
{
	void rtc_initialize(void);
	App_SDMMC_Init();
	rtc_initialize();

	NVIC_DisableIRQ(SDIO_IRQn);
	/* Enable SD/MMC Interrupt */
	NVIC_EnableIRQ(SDIO_IRQn);

	f_mount(0, &Fatfs);		/* Register volume work area (never fails) */

	/* Initialize the mutex if not done already */
	if (mutex_init()) {
		LWIP_DEBUGF(HTTPD_DEBUG, ("DFS: ERROR: Mutex Init!\r\n"));
		return 1;
	}
	return 0;
}

/* Opens the default index html file */
struct fs_file *fs_open_default(void) {
	int hlen;

	struct file_ds *fds;

	struct fs_file *fs;

	fds = (struct file_ds *)mem_malloc(sizeof(*fds));
	if (fds == NULL) {
		DEBUGSTR("Malloc Failure, Out of Memory!\r\n");
		return NULL;
	}
	memset(fds, 0, sizeof(*fds));
	fs = &fds->fs;
	fs->pextension = (void *) fds;	/* Store this for later use */
	hlen = get_http_headers("default.htm", (char *) fds->scratch);
	fs->data = (const char *) fds->scratch;
	memcpy((void *) &fs->data[hlen], (void *) http_index_html, sizeof(http_index_html) - 1);
	fs->len = hlen + sizeof(http_index_html) - 1;
	fs->index = fs->len;
	fs->http_header_included = 1;
	return fs;
}

/* File open function */
struct fs_file *fs_open(const char *name) {
	FRESULT res;
	int hlen;
	struct file_ds *fds;
	struct fs_file *fs;

	/* Too huge to keep in stack, must be protected with mutex */
	static struct file_ds tmpds;

	if (mutex_lock(&open_lock)) {
		LWIP_DEBUGF(HTTPD_DEBUG, ("DFS: ERROR: Mutex Timeout!\r\n"));
		return NULL;
	}
	memset(&tmpds, 0, sizeof(tmpds));
	fds = &tmpds;

	res = f_open(&fds->fi, name, FA_READ);
	if (res) {
		LWIP_DEBUGF(HTTPD_DEBUG, ("DFS: OPEN: File %s does not exist\r\n", name));
		mutex_unlock(&open_lock);
		return NULL;
	}

	fds = (struct file_ds *)mem_malloc(sizeof(*fds));
 	if (fds == NULL) {
		DEBUGSTR("Malloc Failure, Out of Memory!\r\n");
		mutex_unlock(&open_lock);
		return NULL;
	}
	memcpy(fds, &tmpds, sizeof(*fds));
	mutex_unlock(&open_lock);

	fs = &fds->fs;
	fds->fi_valid = 1;
	fs->pextension = (void *) fds;	/* Store this for later use */
	hlen = get_http_headers(name, (char *) fds->scratch);
	fs->data = (const char *) fds->scratch;
	fs->index = hlen;
	fs->len = f_size(&fds->fi) + hlen;
	fs->http_header_included = 1;

	return fs;
}

/* File close function */
void fs_close(struct fs_file *file)
{
	struct file_ds *fds;

	if(file == NULL)
		return;

	fds = (struct file_ds *) file->pextension;

#ifndef BOARD_HITEX_EVA_18504350
	if (fds->fi_valid)
		f_close(&fds->fi);
#endif

 	mem_free(fds);
}

/* File read function */
int fs_read(struct fs_file *file, char *buffer, int count)
{
	uint32_t i = 0;
	struct file_ds *fds = (struct file_ds *) file->pextension;
	if (f_read(&fds->fi, (uint8_t *) buffer, count, &i))
		return 0; /* Error in reading file */
	file->index += i;
	return i;
}

/* Number of bytes left in the file */
int fs_bytes_left(struct fs_file *file)
{
	return file->len - file->index;
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


#ifdef LWIP_DEBUG
#if 0
/* Assert print function */
void assert_printf(char *msg, int line, char *file)
{
	DEBUGOUT("ASSERT: %s at %s:%d\r\n", msg, file, line);
}
#endif

/* LWIP str err function */
const char *lwip_strerr(err_t eno)
{
	return "";
}

#endif

/**
 * @}
 */
