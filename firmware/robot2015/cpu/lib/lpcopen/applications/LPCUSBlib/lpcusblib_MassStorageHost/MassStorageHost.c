/*
 * @brief Mass Storage Host example
 *
 * @note
 * Copyright(C) NXP Semiconductors, 2012
 * Copyright(C) Dean Camera, 2011, 2012
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

#include "MassStorageHost.h"
#include "fsusb_cfg.h"
#include "ff.h"

/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/

/** LPCUSBlib Mass Storage Class driver interface configuration and state information. This structure is
 *  passed to all Mass Storage Class driver functions, so that multiple instances of the same class
 *  within a device can be differentiated from one another.
 */
static USB_ClassInfo_MS_Host_t FlashDisk_MS_Interface = {
	.Config = {
		.DataINPipeNumber       = 1,
		.DataINPipeDoubleBank   = false,

		.DataOUTPipeNumber      = 2,
		.DataOUTPipeDoubleBank  = false,
		.PortNumber = 0,
	},
};

static SCSI_Capacity_t DiskCapacity;
static uint8_t buffer[8 * 1024];

STATIC FATFS fatFS;	/* File system object */
STATIC FIL fileObj;	/* File object */

/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/

/*****************************************************************************
 * Private functions
 ****************************************************************************/

/* Function to spin forever when there is an error */
static void die(FRESULT rc)
{
#if 0
	DEBUGOUT("*******DIE %d*******\r\n", rc);
	while (1) {}/* Spin for ever */
#endif
}

/** Configures the board hardware and chip peripherals for the demo's functionality. */
static void SetupHardware(void)
{
	Board_Init();
	USB_Init(FlashDisk_MS_Interface.Config.PortNumber, USB_MODE_Host);
	/* Hardware Initialization */
	Board_Debug_Init();

	/* Create a stdio stream for the serial port for stdin and stdout */
	Serial_CreateStream(NULL);
}

/* Function to do the read/write to USB Disk */
static void USB_ReadWriteFile(void)
{
	FRESULT rc;		/* Result code */
	int i;
	UINT bw, br;
	uint8_t *ptr;
	char debugBuf[64];
	DIR dir;		/* Directory object */
	FILINFO fno;	/* File information object */

	f_mount(0, &fatFS);		/* Register volume work area (never fails) */

	rc = f_open(&fileObj, "MESSAGE.TXT", FA_READ);
	if (rc) {
		DEBUGOUT("Unable to open MESSAGE.TXT from USB Disk\r\n");
		die(rc);
	}
	else {
		DEBUGOUT("Opened file MESSAGE.TXT from USB Disk. Printing contents...\r\n\r\n");
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

		DEBUGOUT("\r\n\r\nClose the file.\r\n");
		rc = f_close(&fileObj);
		if (rc) {
			die(rc);
		}
	}

	DEBUGOUT("\r\nCreate a new file (hello.txt).\r\n");
	rc = f_open(&fileObj, "HELLO.TXT", FA_WRITE | FA_CREATE_ALWAYS);
	if (rc) {
		die(rc);
	}
	else {

		DEBUGOUT("\r\nWrite a text data. (Hello world!)\r\n");

		rc = f_write(&fileObj, "Hello world!\r\n", 14, &bw);
		if (rc) {
			die(rc);
		}
		else {
			sprintf(debugBuf, "%u bytes written.\r\n", bw);
			DEBUGOUT(debugBuf);
		}
		DEBUGOUT("\r\nClose the file.\r\n");
		rc = f_close(&fileObj);
		if (rc) {
			die(rc);
		}
	}
	DEBUGOUT("\r\nOpen root directory.\r\n");
	rc = f_opendir(&dir, "");
	if (rc) {
		die(rc);
	}
	else {
		DEBUGOUT("\r\nDirectory listing...\r\n");
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
			DEBUGOUT(debugBuf);
		}
		if (rc) {
			die(rc);
		}
	}
	DEBUGOUT("\r\nTest completed.\r\n");
	USB_Host_SetDeviceConfiguration(FlashDisk_MS_Interface.Config.PortNumber, 0);
}

/*****************************************************************************
 * Public functions
 ****************************************************************************/

/** Main program entry point. This routine configures the hardware required by the application, then
 *  calls the filesystem function to read files from USB Disk
 */
int main(void)
{
	SetupHardware();

	DEBUGOUT("Mass Storage Host Demo running.\r\n");

	USB_ReadWriteFile();

	DEBUGOUT("Example completed.\r\n");
	while (1) {}
}

/** Event handler for the USB_DeviceAttached event. This indicates that a device has been attached to the host, and
 *  starts the library USB task to begin the enumeration and USB management process.
 */
void EVENT_USB_Host_DeviceAttached(const uint8_t corenum)
{
	DEBUGOUT(("Device Attached on port %d\r\n"), corenum);
}

/** Event handler for the USB_DeviceUnattached event. This indicates that a device has been removed from the host, and
 *  stops the library USB task management process.
 */
void EVENT_USB_Host_DeviceUnattached(const uint8_t corenum)
{
	DEBUGOUT(("\r\nDevice Unattached on port %d\r\n"), corenum);
}

/** Event handler for the USB_DeviceEnumerationComplete event. This indicates that a device has been successfully
 *  enumerated by the host and is now ready to be used by the application.
 */
void EVENT_USB_Host_DeviceEnumerationComplete(const uint8_t corenum)
{
	uint16_t ConfigDescriptorSize;
	uint8_t  ConfigDescriptorData[512];

	if (USB_Host_GetDeviceConfigDescriptor(corenum, 1, &ConfigDescriptorSize, ConfigDescriptorData,
										   sizeof(ConfigDescriptorData)) != HOST_GETCONFIG_Successful) {
		DEBUGOUT("Error Retrieving Configuration Descriptor.\r\n");
		return;
	}

	FlashDisk_MS_Interface.Config.PortNumber = corenum;
	if (MS_Host_ConfigurePipes(&FlashDisk_MS_Interface,
							   ConfigDescriptorSize, ConfigDescriptorData) != MS_ENUMERROR_NoError) {
		DEBUGOUT("Attached Device Not a Valid Mass Storage Device.\r\n");
		return;
	}

	if (USB_Host_SetDeviceConfiguration(FlashDisk_MS_Interface.Config.PortNumber, 1) != HOST_SENDCONTROL_Successful) {
		DEBUGOUT("Error Setting Device Configuration.\r\n");
		return;
	}

	uint8_t MaxLUNIndex;
	if (MS_Host_GetMaxLUN(&FlashDisk_MS_Interface, &MaxLUNIndex)) {
		DEBUGOUT("Error retrieving max LUN index.\r\n");
		USB_Host_SetDeviceConfiguration(FlashDisk_MS_Interface.Config.PortNumber, 0);
		return;
	}

	DEBUGOUT(("Total LUNs: %d - Using first LUN in device.\r\n"), (MaxLUNIndex + 1));

	if (MS_Host_ResetMSInterface(&FlashDisk_MS_Interface)) {
		DEBUGOUT("Error resetting Mass Storage interface.\r\n");
		USB_Host_SetDeviceConfiguration(FlashDisk_MS_Interface.Config.PortNumber, 0);
		return;
	}

	SCSI_Request_Sense_Response_t SenseData;
	if (MS_Host_RequestSense(&FlashDisk_MS_Interface, 0, &SenseData) != 0) {
		DEBUGOUT("Error retrieving device sense.\r\n");
		USB_Host_SetDeviceConfiguration(FlashDisk_MS_Interface.Config.PortNumber, 0);
		return;
	}

// 	if (MS_Host_PreventAllowMediumRemoval(&FlashDisk_MS_Interface, 0, true)) {
// 		DEBUGOUT("Error setting Prevent Device Removal bit.\r\n");
// 		USB_Host_SetDeviceConfiguration(FlashDisk_MS_Interface.Config.PortNumber, 0);
// 		return;
// 	}

	SCSI_Inquiry_Response_t InquiryData;
	if (MS_Host_GetInquiryData(&FlashDisk_MS_Interface, 0, &InquiryData)) {
		DEBUGOUT("Error retrieving device Inquiry data.\r\n");
		USB_Host_SetDeviceConfiguration(FlashDisk_MS_Interface.Config.PortNumber, 0);
		return;
	}

	/* DEBUGOUT("Vendor \"%.8s\", Product \"%.16s\"\r\n", InquiryData.VendorID, InquiryData.ProductID); */

	DEBUGOUT("Mass Storage Device Enumerated.\r\n");
}

/** Event handler for the USB_HostError event. This indicates that a hardware error occurred while in host mode. */
void EVENT_USB_Host_HostError(const uint8_t corenum, const uint8_t ErrorCode)
{
	USB_Disable(corenum, USB_MODE_Host);

	DEBUGOUT(("Host Mode Error\r\n"
			  " -- Error port %d\r\n"
			  " -- Error Code %d\r\n" ), corenum, ErrorCode);

	for (;; ) {}
}

/** Event handler for the USB_DeviceEnumerationFailed event. This indicates that a problem occurred while
 *  enumerating an attached USB device.
 */
void EVENT_USB_Host_DeviceEnumerationFailed(const uint8_t corenum,
											const uint8_t ErrorCode,
											const uint8_t SubErrorCode)
{
	DEBUGOUT(("Dev Enum Error\r\n"
			  " -- Error port %d\r\n"
			  " -- Error Code %d\r\n"
			  " -- Sub Error Code %d\r\n"
			  " -- In State %d\r\n" ),
			 corenum, ErrorCode, SubErrorCode, USB_HostState[corenum]);

}

/* Get the disk data structure */
DISK_HANDLE_T *FSUSB_DiskInit(void)
{
	return &FlashDisk_MS_Interface;
}

/* Wait for disk to be inserted */
int FSUSB_DiskInsertWait(DISK_HANDLE_T *hDisk)
{
	while (USB_HostState[hDisk->Config.PortNumber] != HOST_STATE_Configured) {
		MS_Host_USBTask(hDisk);
		USB_USBTask(hDisk->Config.PortNumber, USB_MODE_Host);
	}
	return 1;
}

/* Disk acquire function that waits for disk to be ready */
int FSUSB_DiskAcquire(DISK_HANDLE_T *hDisk)
{
	DEBUGOUT("Waiting for ready...");
	for (;; ) {
		uint8_t ErrorCode = MS_Host_TestUnitReady(hDisk, 0);

		if (!(ErrorCode)) {
			break;
		}

		/* Check if an error other than a logical command error (device busy) received */
		if (ErrorCode != MS_ERROR_LOGICAL_CMD_FAILED) {
			DEBUGOUT("Failed\r\n");
			USB_Host_SetDeviceConfiguration(hDisk->Config.PortNumber, 0);
			return 0;
		}
	}
	DEBUGOUT("Done.\r\n");

	if (MS_Host_ReadDeviceCapacity(hDisk, 0, &DiskCapacity)) {
		DEBUGOUT("Error retrieving device capacity.\r\n");
		USB_Host_SetDeviceConfiguration(hDisk->Config.PortNumber, 0);
		return 0;
	}

	DEBUGOUT(("%lu blocks of %lu bytes.\r\n"), DiskCapacity.Blocks, DiskCapacity.BlockSize);
	return 1;
}

/* Get sector count */
uint32_t FSUSB_DiskGetSectorCnt(DISK_HANDLE_T *hDisk)
{
	return DiskCapacity.Blocks;
}

/* Get Block size */
uint32_t FSUSB_DiskGetSectorSz(DISK_HANDLE_T *hDisk)
{
	return DiskCapacity.BlockSize;
}

/* Read sectors */
int FSUSB_DiskReadSectors(DISK_HANDLE_T *hDisk, void *buff, uint32_t secStart, uint32_t numSec)
{
	if (MS_Host_ReadDeviceBlocks(hDisk, 0, secStart, numSec, DiskCapacity.BlockSize, buff)) {
		DEBUGOUT("Error reading device block.\r\n");
		USB_Host_SetDeviceConfiguration(FlashDisk_MS_Interface.Config.PortNumber, 0);
		return 0;
	}
	return 1;
}

/* Write Sectors */
int FSUSB_DiskWriteSectors(DISK_HANDLE_T *hDisk, void *buff, uint32_t secStart, uint32_t numSec)
{
	if (MS_Host_WriteDeviceBlocks(hDisk, 0, secStart, numSec, DiskCapacity.BlockSize, buff)) {
		DEBUGOUT("Error writing device block.\r\n");
		return 0;
	}
	return 1;
}

/* Disk ready function */
int FSUSB_DiskReadyWait(DISK_HANDLE_T *hDisk, int tout)
{
	volatile int i = tout * 100;
	while (i--) {	/* Just delay */
	}
	return 1;
}
