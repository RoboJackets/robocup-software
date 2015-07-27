/*
 * @brief DFU utility (programmer) streaming code
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

#include "app_usbd_cfg.h"
#include "usbd_desc.h"
#include "usbd_mscuser.h"
#include "usbd.h"
#include "usbd_core.h"
#include "usbd_hw.h"
#include "usbd_mscuser.h"
#include "usbd_rom_api.h"

#include "board.h"
#include "dfuutil_programming_api.h"

#include "string.h"

/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/

/* USB handle for ROM driver */
static USBD_HANDLE_T hUsb;

/* Saved pointer to region data for programming algorithm */
static DFUPROG_REGION_T *dfuprog_regions;

/* Current status of the program and last host command */
static volatile uint32_t currStatus;
static volatile uint32_t hostCmd;

/* Address and size for the current command */
static volatile uint32_t currCmdAddr, currCmdSize;

/* Time counter in 1mS ticks */
static volatile uint32_t u32Milliseconds;

/* Size of USB debug message buffer */
#define USBMSGBUFFSIZE 1024

/* USB debug message buffer and pointers */
static char usbBuff[USBMSGBUFFSIZE];
static volatile int usbStrIn, usbStrOut;

/* Buffer for DFU output (to host) */
static uint32_t dfuIn[((sizeof(DFU_TOHOST_PACKETHDR_T) + 64 + 4096) /
					   sizeof(uint32_t))];

/* Buffer for DFU input (from host) */
static uint32_t dfuOut[((sizeof(DFU_FROMHOST_PACKETHDR_T) + 4096 +
						 sizeof(uint32_t)) / sizeof(uint32_t))];

/* Program data buffer (to free up USB buffer) */
static uint32_t dfuProgBuff[(4096 / sizeof(uint32_t))];

/* IN packet indexing and size */
static volatile uint32_t inPktSize, inPktSizeIdx, outPktSizeIdx, progSize;

/* Be careful with this number, as the linker may be setup to use ranges
   just outside this area's size */
#define USBROMBUFFSIZE 0x2000

/* Temporary string buffer */
static char tmpStr[64];

/* Quiet mode flag, used with the parameter option to enable verbose
   debug messages */
static bool verifyDebug = true;

/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/

extern uint8_t USB_DeviceDescriptor[];
extern uint8_t USB_FsConfigDescriptor[];
extern uint8_t USB_HsConfigDescriptor[];
extern uint8_t USB_StringDescriptor[];
extern uint8_t USB_DeviceQualifier[];

volatile uint32_t dfu_done_algo = 0, dfu_detach_algo = 0;

/*****************************************************************************
 * Private functions
 ****************************************************************************/

/* Delay until ticks expired */
static void tickDelayMS(int ticks)
{
	u32Milliseconds = ticks;
	while (u32Milliseconds) {}
}

/* Sets up debug message buffering for USB status messages */
static void usbDebugSetup(void)
{
	/* Just reset pointers to start of buffer */
	usbStrIn = usbStrOut = 0;
}

/* Gets message data for a DFU status transfer */
static int usbDebugFill(void *buff, int max)
{
	int ssize = 0;

	if (usbStrIn != usbStrOut) {
		/* Largest size to send is until the end of the buffer or
		   the passed maximum size */
		if (usbStrOut > usbStrIn) {
			/* Can send up to the end of the buffer */
			ssize = USBMSGBUFFSIZE - usbStrOut;
		}
		else {
			/* Can send up to the end of input */
			ssize = usbStrIn - usbStrOut;
		}

		/* Limit size to input */
		if (ssize > max) {
			ssize = max;
		}

		/* Copy data */
		memmove(buff, &usbBuff[usbStrOut], ssize);

		/* Update start of output and next output index */
		usbStrOut += ssize;
		if (usbStrOut >= USBMSGBUFFSIZE) {
			usbStrOut = 0;
		}
	}

	return ssize;
}

/* Appends data to a USB buffer */
static int usbDFUAppend(char *ptr, void *data, uint32_t bytes)
{
	memmove(ptr, data, bytes);

	return bytes;
}

/* Build the response header */
static int usbDFUBuildStatus(void *buffer)
{
	char *ptr = (char *) buffer;
	int strbytes, rsvd, bytes;

	/* Place string in buffer if needed */
	strbytes = usbDebugFill((ptr + 16), 64);

	/* Create header */
	bytes = usbDFUAppend((ptr + 0), (void *) &hostCmd, 4);
	bytes += usbDFUAppend((ptr + 4), (void *) &currStatus, 4);
	bytes += usbDFUAppend((ptr + 8), &strbytes, 4);

	/* Field is marked as reserved, but it is being used for the
	   DFU buffer size */
	rsvd = dfuprog_regions->buffer_size;
	bytes += usbDFUAppend((ptr + 12), &rsvd, 4);/* Reserved[1] field */

	/* Allocate some space for debug message if needed */
	if (strbytes != 0) {
		bytes += 64;
	}

	return bytes;
}

/* Build programming algo status */
static int usbDFUReturnStatus(void *buffer)
{
	return usbDFUBuildStatus(buffer);
}

/* Respond to DFU_HOSTCMD_SETDEBUG command */
static void usbDFUSetVerbose(uint32_t addr)
{
	/* Set quiet or verbose mode based on address field */
	hostCmd = DFU_HOSTCMD_SETDEBUG;

	if (addr) {
		verifyDebug = false;
	}
	else {
		verifyDebug = true;
	}
}

/* Respond to DFU_HOSTCMD_ERASE_ALL command */
static void usbDFUEraseAll(void)
{
	/* Set erase start state so it starts in the background. */
	currStatus = DFU_OPSTS_ERASE_ALL_ST;
	hostCmd = DFU_HOSTCMD_ERASE_ALL;
}

/* Respond to DFU_HOSTCMD_ERASE_REGION command */
static void usbDFUEraseRegion(uint32_t addr, uint32_t size)
{
	/* Set erase start state so it starts in the background. */
	currStatus = DFU_OPSTS_ERASE_ST;
	hostCmd = DFU_HOSTCMD_ERASE_REGION;

	/* Save erase address and size */
	currCmdAddr = addr;
	currCmdSize = size;
}

/* Respond to DFU_HOSTCMD_PROGRAM command */
static void usbDFUProgRegion(uint32_t addr, uint32_t size)
{
	/* Set erase start state so it starts in the background. */
	currStatus = DFU_OPSTS_PROG_STREAM;
	hostCmd = DFU_HOSTCMD_PROGRAM;

	/* Save program address and size */
	currCmdAddr = addr;
	currCmdSize = size;
}

/* Respond to DFU_HOSTCMD_READBACK command */
static void usbDFUReadRegion(uint32_t addr, uint32_t size)
{
	/* Set erase start state so it starts in the background. */
	currStatus = DFU_OPSTS_READBUSY;
	hostCmd = DFU_HOSTCMD_READBACK;

	/* Save read address and size */
	currCmdAddr = addr;
	currCmdSize = size;
}

/* Respond to DFU_HOSTCMD_RESET command */
static void usbDFUReset(void)
{
	/* Will reset in background. */
	currStatus = DFU_OPSTS_RESET;
	hostCmd = DFU_HOSTCMD_RESET;
}

/* Respond to DFU_HOSTCMD_EXECUTE command */
static void usbDFUExecute(uint32_t addr)
{
	/* Will reset in background. */
	currStatus = DFU_OPSTS_EXEC;
	hostCmd = DFU_HOSTCMD_EXECUTE;

	currCmdAddr = addr;
}

/* Will set dfu_detach_algo flag when USB detaches */
static void dfu_detach(USBD_HANDLE_T hUsb)
{
	dfu_detach_algo = 1;
}

/* Will set dfu_done_algo flag when USB DFU download is complete (ZLP received) */
static void dfu_done(void)
{
	dfu_done_algo = 1;
}

void USB0_IRQHandler(void)
{
	USBD_API->hw->ISR(hUsb);
}

void USB1_IRQHandler(void)
{
	USBD_API->hw->ISR(hUsb);
}

/* Checks header */
static bool checkMagicHeader(DFU_FROMHOST_PACKETHDR_T *pOutHdr)
{
	bool checked = false;

	if ((pOutHdr->magic != DFUPROG_VALIDVAL) && (checked == false)) {
		/* Let it keep running, but send a warning */
		usbDebug("DFU Utility and programming algorithm have different versions\n");
		checked = true;
	}

	return true;
}

/* Handles OUT (from host) packets and requests from host */
static uint8_t dfu_wr(uint32_t block_num, uint8_t * *pBuff, uint32_t length,
					  uint8_t *bwPollTimeout)
{
	DFU_FROMHOST_PACKETHDR_T *pOutHdr;
	uint32_t addr, size;

	/* Reset buffer on length = 0 */
	if (length != 0) {
		/* Concatenate packets */
		*pBuff += length;
		outPktSizeIdx += length;
	}

	/* Hack for no ZLP on end of aligned DFU transfer */
	if (currStatus == DFU_OPSTS_PROG_STREAM) {
		if (length == 0) {
			outPktSizeIdx = 0;
			*pBuff = (uint8_t *) dfuProgBuff;
		}
		else if ((outPktSizeIdx == dfuprog_regions->buffer_size) ||
				 (outPktSizeIdx == currCmdSize)) {
			currStatus = DFU_OPSTS_PROG;
			progSize = outPktSizeIdx;
			currCmdSize -= outPktSizeIdx;
			outPktSizeIdx = 0;
			*pBuff = (uint8_t *) dfuIn;
		}
	}
	else if (length != 0) {
		/* Parse packet header, not necessarily used on all receives */
		pOutHdr = (DFU_FROMHOST_PACKETHDR_T *) dfuIn;
		addr = pOutHdr->addr;
		size = pOutHdr->size;
		checkMagicHeader(pOutHdr);

		/* Process based on current state */
		switch (pOutHdr->hostCmd) {
		case DFU_HOSTCMD_SETDEBUG:
			usbDFUSetVerbose(addr);
			break;

		case DFU_HOSTCMD_ERASE_ALL:
			usbDFUEraseAll();
			break;

		case DFU_HOSTCMD_ERASE_REGION:
			usbDFUEraseRegion(addr, size);
			break;

		case DFU_HOSTCMD_PROGRAM:
			/* Only called in stream mode */
			usbDFUProgRegion(addr, size);
			outPktSizeIdx = 0;
			*pBuff = (uint8_t *) dfuProgBuff;
			return DFU_STATUS_OK;

		case DFU_HOSTCMD_READBACK:
			usbDFUReadRegion(addr, size);
			break;

		case DFU_HOSTCMD_RESET:
			usbDFUReset();
			break;

		case DFU_HOSTCMD_EXECUTE:
			usbDFUExecute(addr);
			break;

		default:
			/* Unknown */
			usbDebug("Unknown command\n");
			currStatus = DFU_OPSTS_ERRUN;
			break;
		}

		outPktSizeIdx = 0;
		*pBuff = (uint8_t *) dfuIn;
	}
	else {
		outPktSizeIdx = 0;
		*pBuff = (uint8_t *) dfuIn;
	}

	return DFU_STATUS_OK;
}

/* DFU IN (to host) state machine for status polling */
static uint32_t dfu_rd(uint32_t block_num, uint8_t * *pBuff, uint32_t length)
{
	uint8_t *pBuf = (uint8_t *) *pBuff;
	uint8_t *ptrCurr;

	if (length == 0) {
		inPktSize = 0;
	}

	/* Task based on current state */
	switch (currStatus) {
	case DFU_OPSTS_IDLE:
	case DFU_OPSTS_ERRER:
	case DFU_OPSTS_PROGER:
	case DFU_OPSTS_READER:
	case DFU_OPSTS_ERRUN:
	case DFU_OPSTS_READBUSY:
	case DFU_OPSTS_READTRIG:
	case DFU_OPSTS_ERASE_ALL_ST:
	case DFU_OPSTS_ERASE_ST:
	case DFU_OPSTS_ERASE:
	case DFU_OPSTS_PROG:
	case DFU_OPSTS_PROG_STREAM:
	case DFU_OPSTS_RESET:
	case DFU_OPSTS_EXEC:
	case DFU_OPSTS_LOOP:
		/* All these states return only status */
		if (inPktSize == 0) {
			/* Build status response */
			inPktSize = usbDFUReturnStatus((void *) dfuOut);
			inPktSizeIdx = 0;
			ptrCurr = (uint8_t *) dfuOut;
		}

		if (inPktSize != 0) {
			if (length > inPktSize) {
				length = inPktSize;
			}

			if (currStatus == DFU_OPSTS_READTRIG) {
				/* Switch to read ready state on trnasfer completion */
				currStatus = DFU_OPSTS_READREADY;
			}

			memmove(pBuf, &ptrCurr[inPktSizeIdx], length);
			inPktSize -= length;
			inPktSizeIdx += length;
		}
		break;

	case DFU_OPSTS_READREADY:
		if (inPktSize == 0) {
			inPktSize = currCmdSize;
			if (inPktSize > dfuprog_regions->buffer_size) {
				inPktSize = dfuprog_regions->buffer_size;
			}
			currCmdSize -= inPktSize;
			inPktSizeIdx = 0;
			ptrCurr = (uint8_t *) dfuProgBuff;
		}

		if (inPktSize != 0) {
			if (length > inPktSize) {
				length = inPktSize;
			}

			memmove(pBuf, &ptrCurr[inPktSizeIdx], length);
			inPktSize -= length;
			inPktSizeIdx += length;
		}

		if (inPktSize == 0) {
			/* Next block */
			if (currCmdSize == 0) {
				/* Done with transfer */
				currStatus = DFU_OPSTS_IDLE;
			}
			else {
				currStatus = DFU_OPSTS_READBUSY;
			}
		}
		break;

	default:
		/* No other cases */
		usbDebug("Bad programming algorithm state\n");
		currStatus = DFU_OPSTS_ERRUN;
		length = dfu_rd(block_num, pBuff, length);
		break;
	}

	return length;
}

/* USB DFU init via ROM driver */
static ErrorCode_t usb_dfu_init(USBD_HANDLE_T hUsb,
								USB_INTERFACE_DESCRIPTOR *pIntfDesc,
								uint32_t *mem_base,
								uint32_t *mem_size)
{
	USBD_DFU_INIT_PARAM_T dfu_param;
	ErrorCode_t ret = LPC_OK;

	memset((void *) &dfu_param, 0, sizeof(USBD_DFU_INIT_PARAM_T));
	dfu_param.mem_base = *mem_base;
	dfu_param.mem_size = *mem_size;
	/* DFU paramas */
	dfu_param.wTransferSize = USB_DFU_XFER_SIZE;

	if ((pIntfDesc == 0) ||
		(pIntfDesc->bInterfaceClass != USB_DEVICE_CLASS_APP) ||
		(pIntfDesc->bInterfaceSubClass != USB_DFU_SUBCLASS) ) {
		return ERR_FAILED;
	}

	dfu_param.intf_desc = (uint8_t *) pIntfDesc;
	/* user defined functions */
	dfu_param.DFU_Write = dfu_wr;	/* From host */
	dfu_param.DFU_Read = dfu_rd;/* To host */
	dfu_param.DFU_Done = dfu_done;
	dfu_param.DFU_Detach = dfu_detach;

	ret = USBD_API->dfu->init(hUsb, &dfu_param, DFU_STATE_dfuIDLE);
	/* update memory variables */
	*mem_base = dfu_param.mem_base;
	*mem_size = dfu_param.mem_size;

	return ret;
}

/* Sets up USB for DFU operation using boot ROM */
static uint32_t algo_dfu_setup(void)
{
	USBD_API_INIT_PARAM_T usb_param;
	USB_CORE_DESCS_T desc;
	ErrorCode_t ret;
	USB_INTERFACE_DESCRIPTOR *pIntfDesc;

	/* Initilize call back structures */
	memset((void *) &usb_param, 0, sizeof(USBD_API_INIT_PARAM_T));

	/* USB_EPWR power saving mode will be enabled if the boot ROM
	   was booted from USB1 */
	if ((LPC_SCU->SFSUSB & (1 << 4)) == 0) {
		usb_param.usb_reg_base = LPC_USB0_BASE;
	}
	else {
		usb_param.usb_reg_base = LPC_USB1_BASE;
	}

	usb_param.max_num_ep = 6;
	usb_param.mem_base = 0x20000000;
	usb_param.mem_size = USBROMBUFFSIZE;

	/* Set the USB descriptors */
	desc.device_desc = USB_DeviceDescriptor;
	desc.string_desc = USB_StringDescriptor;
	desc.full_speed_desc = USB_FsConfigDescriptor;
	desc.high_speed_desc = USB_HsConfigDescriptor;
	desc.device_qualifier = USB_DeviceQualifier;

	/* USB Initialization */
	ret = USBD_API->hw->Init(&hUsb, &desc, &usb_param);
	if (ret == LPC_OK) {
		pIntfDesc = (USB_INTERFACE_DESCRIPTOR *) ((uint32_t) desc.high_speed_desc + USB_CONFIGUARTION_DESC_SIZE);
		ret = usb_dfu_init(hUsb, pIntfDesc, &usb_param.mem_base, &usb_param.mem_size);
		if (ret != LPC_OK) {
			return 0;
		}

		NVIC_EnableIRQ(USB0_IRQn);
		NVIC_EnableIRQ(USB1_IRQn);
	}

	USBD_API->hw->Connect(hUsb, 1);

	return 1;
}

/* Reset chip via the RGU */
static void lpc18xx43xx_sys_reset(void)
{
	/* Reset core via RGU */
	while (1) {
		Chip_RGU_TriggerReset(RGU_CORE_RST);
	}
}

/**
 * @brief	Sets up the USB PLL
 * @return	None
 */
static void usbPllSetup(void)
{
	/* Nothing to do here, the PLL settings are inherited from the
	   boot ROM on USB boot */
}

/* DFU processing entry point */
static void dfu_util_process(void)
{
	PFV pfunc;
	int i, blks;

	/* Initialize programming interfaces and get device gemometry */
	dfuprog_regions = algo_flash_init();

	/* Show regions */
	sprintf(tmpStr, "Number of memory regions : %d\n", dfuprog_regions->num_regions);
	usbDebug(tmpStr);
	for (i = 0; i < dfuprog_regions->num_regions; i++) {
		sprintf(tmpStr, "Region #%d @ 0x%p, size %p bytes\n", i,
				(void *) dfuprog_regions->pregions[i].region_addr,
				(void *) dfuprog_regions->pregions[i].region_size);
		usbDebug(tmpStr);
	}

	/* Before starting USB, setup USB state for configuration */
	currStatus = DFU_OPSTS_IDLE;

	/* Setup DFU USB mode and enumerate */
	if (algo_dfu_setup() == 0) {
		/* DFU failed, so status can't be returned. Attempt a reset */
		USBD_API->hw->Connect(hUsb, 0);
		currStatus = DFU_OPSTS_ERRUN;
		tickDelayMS(500);
	}

	/* Processing loop for background programming operation */
	while (1) {
		/* Background processing loop */
		switch (currStatus) {
		case DFU_OPSTS_IDLE:
		/* Do nothing when idle */
		case DFU_OPSTS_ERRER:
		case DFU_OPSTS_READER:
		case DFU_OPSTS_PROGER:
		case DFU_OPSTS_ERRUN:
		/* Error states are handled in the interrupt handler, so
		   there is nothing to do here. */
		case DFU_OPSTS_READREADY:
		case DFU_OPSTS_READTRIG:
			__WFI();
			break;

		case DFU_OPSTS_READBUSY:
			blks = currCmdSize;
			if (blks > dfuprog_regions->buffer_size) {
				blks = dfuprog_regions->buffer_size;
			}
			if (dfuprog_regions->pprogalgos->read(dfuProgBuff, currCmdAddr,
												  blks) == 0) {
				currStatus = DFU_OPSTS_READER;
			}
			else {
				/* Read data and return to ready state on completion */
				currStatus = DFU_OPSTS_READTRIG;
				currCmdAddr += blks;
			}
			break;

		case DFU_OPSTS_ERASE_ALL_ST:
			/* Starting a full erase operation */
			currStatus = DFU_OPSTS_ERASE;
			if (dfuprog_regions->pprogalgos->erase_all() == 0) {
				currStatus = DFU_OPSTS_ERRER;
			}
			break;

		case DFU_OPSTS_ERASE_ST:
			/* Starting a region erase operation */
			currStatus = DFU_OPSTS_ERASE;
			if (dfuprog_regions->pprogalgos->erase_region(currCmdAddr,
														  currCmdSize) == 0) {
				currStatus = DFU_OPSTS_ERRER;
			}
			break;

		case DFU_OPSTS_ERASE:
			/* Since the erase function is blocking, this state indicatea a
			   successful erase or program operation. Switch back to idle state. */
			currStatus = DFU_OPSTS_IDLE;
			break;

		case DFU_OPSTS_PROG_STREAM:
			/* Nothing to do in this state */
			__WFI();
			break;

		case DFU_OPSTS_PROG:
			if (progSize == 0) {
				/* Done, go back to idle */
				currStatus = DFU_OPSTS_IDLE;
			}
			else {
				if (dfuprog_regions->pprogalgos->write((void *) dfuProgBuff,
													   currCmdAddr, progSize) != progSize) {
					currStatus = DFU_OPSTS_PROGER;
				}
				else if ((progSize < dfuprog_regions->buffer_size) ||
					(currCmdSize == 0)) {
					currStatus = DFU_OPSTS_IDLE;
				}
				else {
					currStatus = DFU_OPSTS_PROG_STREAM;
					currCmdAddr += progSize;
				}
			}
			break;

		case DFU_OPSTS_RESET:
			/* Reset the chip */
			tickDelayMS(100);
			USBD_API->hw->Connect(hUsb, 0);
			NVIC_DisableIRQ(SysTick_IRQn);
			lpc18xx43xx_sys_reset();
			break;

		case DFU_OPSTS_EXEC:
			/* Jump to a address */
			tickDelayMS(100);
			pfunc = (PFV) currCmdAddr;
			USBD_API->hw->Connect(hUsb, 0);
			NVIC_DisableIRQ(SysTick_IRQn);
			pfunc();
			break;

		case DFU_OPSTS_LOOP:
			/* Dead loop, happens on an error */
			USBD_API->hw->Connect(hUsb, 0);
			while (1) {}
		}
	}
}

/*****************************************************************************
 * Public functions
 ****************************************************************************/

/**
 * @brief	SysTick 1mS tick handler
 * @return	None
 */
void SysTick_Handler(void)
{
	if (u32Milliseconds > 0) {
		u32Milliseconds--;
	}
}

/* Queues a message for DFU status transfer */
void usbDebug(char *tmp)
{
	if (verifyDebug == true) {
		/* Wrap without limits */
		while (*tmp != '\0') {
			usbBuff[usbStrIn] = *tmp;
			usbStrIn++;
			if (usbStrIn >= USBMSGBUFFSIZE) {
				usbStrIn = 0;
			}
			tmp++;
		}
	}
}

/**
 * @brief	Main program body
 * @return	Always returns 0 (device reset prior to return)
 */
int main(void)
{
	/* Updates SystemCoreClock global var with current clock speed */
	SystemCoreClockUpdate();

	/* Setup and enable USB PLL and PHY */
	usbPllSetup();

	/* Setup 1mS tick source for timing */
	SysTick_Config(Chip_Clock_GetRate(CLK_MX_MXCORE) / 1000);

	/* Start debug output buffering for for DFU Utility */
	usbDebugSetup();
	usbDebug("LPC18xx/43xx DFUSec programming API tool\n");
	usbDebug("Build date: " __DATE__ ":" __TIME__ "\n");

	/* Will stay in processing until a reset is requested by host */
	dfu_util_process();

	/* dfu_util_process() shouldn't exit, so reset the chip/board
	   if it does. */
	lpc18xx43xx_sys_reset();

	return 0;
}
