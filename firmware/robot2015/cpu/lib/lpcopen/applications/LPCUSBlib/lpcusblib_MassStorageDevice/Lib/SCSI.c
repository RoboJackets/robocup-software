/*
 * @brief SCSI command processing routines, for SCSI commands issues by the host
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

#define  INCLUDE_FROM_SCSI_C
#include "SCSI.h"

/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/

/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/

/** Structure to hold the SCSI response data to a SCSI INQUIRY command. This gives information about the device's
 *  features and capabilities.
 */
static const SCSI_Inquiry_Response_t InquiryData = {
	.DeviceType          = DEVICE_TYPE_BLOCK,
	.PeripheralQualifier = 0,

	.Removable           = true,

	.Version             = 0,

	.ResponseDataFormat  = 2,
	.NormACA             = false,
	.TrmTsk              = false,
	.AERC                = false,

	.AdditionalLength    = 0x1F,

	.SoftReset           = false,
	.CmdQue              = false,
	.Linked              = false,
	.Sync                = false,
	.WideBus16Bit        = false,
	.WideBus32Bit        = false,
	.RelAddr             = false,

	.VendorID            = "NXP",
	.ProductID           = "Dataflash Disk",
	.RevisionID          = {'0', '.', '0', '0'},
};

/** Structure to hold the sense data for the last issued SCSI command, which is returned to the host after a SCSI REQUEST SENSE
 *  command is issued. This gives information on exactly why the last command failed to complete.
 */
static SCSI_Request_Sense_Response_t SenseData = {
	.ResponseCode        = 0x70,
	.AdditionalLength    = 0x0A,
};

/** Under development, not working yet. */
#ifdef CFG_SDCARD
uint8_t *disk_cache = (uint8_t *) 0x20000000;
static uint32_t MSC_Get_Block_Count(void)
{
	return (uint32_t) Chip_SDMMC_GetDeviceBlocks(LPC_SDMMC);
}

#endif

/*****************************************************************************
 * Private functions
 ****************************************************************************/

/*****************************************************************************
 * Public functions
 ****************************************************************************/

/** Main routine to process the SCSI command located in the Command Block Wrapper read from the host. This dispatches
 *  to the appropriate SCSI command handling routine if the issued command is supported by the device, else it returns
 *  a command failure due to a ILLEGAL REQUEST.
 */
bool SCSI_DecodeSCSICommand(USB_ClassInfo_MS_Device_t *const MSInterfaceInfo)
{
	bool CommandSuccess = false;

	/* Run the appropriate SCSI command hander function based on the passed command */
	switch (MSInterfaceInfo->State.CommandBlock.SCSICommandData[0]) {
	case SCSI_CMD_INQUIRY:
		CommandSuccess = SCSI_Command_Inquiry(MSInterfaceInfo);
		break;

	case SCSI_CMD_REQUEST_SENSE:
		CommandSuccess = SCSI_Command_Request_Sense(MSInterfaceInfo);
		break;

	case SCSI_CMD_READ_CAPACITY_10:
		CommandSuccess = SCSI_Command_Read_Capacity_10(MSInterfaceInfo);
		break;

	case SCSI_CMD_SEND_DIAGNOSTIC:
		CommandSuccess = SCSI_Command_Send_Diagnostic(MSInterfaceInfo);
		break;

	case SCSI_CMD_WRITE_10:
		CommandSuccess = SCSI_Command_ReadWrite_10(MSInterfaceInfo, DATA_WRITE);
		break;

	case SCSI_CMD_READ_10:
		CommandSuccess = SCSI_Command_ReadWrite_10(MSInterfaceInfo, DATA_READ);
		break;

	case SCSI_CMD_MODE_SENSE_6:
		CommandSuccess = SCSI_Command_ModeSense_6(MSInterfaceInfo);
		break;

	case SCSI_CMD_TEST_UNIT_READY:
	case SCSI_CMD_PREVENT_ALLOW_MEDIUM_REMOVAL:
	case SCSI_CMD_VERIFY_10:
		/* These commands should just succeed, no handling required */
		CommandSuccess = true;
		MSInterfaceInfo->State.CommandBlock.DataTransferLength = 0;
		break;

	default:
		/* Update the SENSE key to reflect the invalid command */
		SCSI_SET_SENSE(SCSI_SENSE_KEY_ILLEGAL_REQUEST,
					   SCSI_ASENSE_INVALID_COMMAND,
					   SCSI_ASENSEQ_NO_QUALIFIER);
		break;
	}

	/* Check if command was successfully processed */
	if (CommandSuccess) {
		SCSI_SET_SENSE(SCSI_SENSE_KEY_GOOD,
					   SCSI_ASENSE_NO_ADDITIONAL_INFORMATION,
					   SCSI_ASENSEQ_NO_QUALIFIER);

		return true;
	}

	return false;
}

/** Command processing for an issued SCSI INQUIRY command. This command returns information about the device's features
 *  and capabilities to the host.
 */
static bool SCSI_Command_Inquiry(USB_ClassInfo_MS_Device_t *const MSInterfaceInfo)
{
	uint16_t AllocationLength  = SwapEndian_16(*(uint16_t *) &MSInterfaceInfo->State.CommandBlock.SCSICommandData[3]);
	uint16_t BytesTransferred  = MIN(AllocationLength, sizeof(InquiryData));

	/* Only the standard INQUIRY data is supported, check if any optional INQUIRY bits set */
	if ((MSInterfaceInfo->State.CommandBlock.SCSICommandData[1] & ((1 << 0) | (1 << 1))) ||
		MSInterfaceInfo->State.CommandBlock.SCSICommandData[2]) {
		/* Optional but unsupported bits set - update the SENSE key and fail the request */
		SCSI_SET_SENSE(SCSI_SENSE_KEY_ILLEGAL_REQUEST,
					   SCSI_ASENSE_INVALID_FIELD_IN_CDB,
					   SCSI_ASENSEQ_NO_QUALIFIER);

		return false;
	}

	Endpoint_Write_Stream_LE(MSInterfaceInfo->Config.PortNumber, &InquiryData, BytesTransferred, NULL);

	/* Pad out remaining bytes with 0x00 */
	Endpoint_Null_Stream(MSInterfaceInfo->Config.PortNumber, (AllocationLength - BytesTransferred), NULL);

	/* Finalize the stream transfer to send the last packet */
	Endpoint_ClearIN(MSInterfaceInfo->Config.PortNumber);

	/* Succeed the command and update the bytes transferred counter */
	MSInterfaceInfo->State.CommandBlock.DataTransferLength -= BytesTransferred;

	return true;
}

/** Command processing for an issued SCSI REQUEST SENSE command. This command returns information about the last issued command,
 *  including the error code and additional error information so that the host can determine why a command failed to complete.
 */
static bool SCSI_Command_Request_Sense(USB_ClassInfo_MS_Device_t *const MSInterfaceInfo)
{
	uint8_t  AllocationLength = MSInterfaceInfo->State.CommandBlock.SCSICommandData[4];
	uint8_t  BytesTransferred = MIN(AllocationLength, sizeof(SenseData));

	Endpoint_Write_Stream_LE(MSInterfaceInfo->Config.PortNumber, &SenseData, BytesTransferred, NULL);
	Endpoint_Null_Stream(MSInterfaceInfo->Config.PortNumber, (AllocationLength - BytesTransferred), NULL);
	Endpoint_ClearIN(MSInterfaceInfo->Config.PortNumber);

	/* Succeed the command and update the bytes transferred counter */
	MSInterfaceInfo->State.CommandBlock.DataTransferLength -= BytesTransferred;

	return true;
}

/** Command processing for an issued SCSI READ CAPACITY (10) command. This command returns information about the device's capacity
 *  on the selected Logical Unit (drive), as a number of OS-sized blocks.
 */
static bool SCSI_Command_Read_Capacity_10(USB_ClassInfo_MS_Device_t *const MSInterfaceInfo)
{
	/** Under development, not working yet. */
#ifdef CFG_SDCARD
	uint32_t LastBlockAddressInLUN = MSC_Get_Block_Count() - 1;
#else
	uint32_t LastBlockAddressInLUN = (LUN_MEDIA_BLOCKS - 1);
#endif
	uint32_t MediaBlockSize        = VIRTUAL_MEMORY_BLOCK_SIZE;

	Endpoint_Write_Stream_BE(MSInterfaceInfo->Config.PortNumber,
							 &LastBlockAddressInLUN,
							 sizeof(LastBlockAddressInLUN),
							 NULL);
	Endpoint_Write_Stream_BE(MSInterfaceInfo->Config.PortNumber, &MediaBlockSize, sizeof(MediaBlockSize), NULL);
	Endpoint_ClearIN(MSInterfaceInfo->Config.PortNumber);

	/* Succeed the command and update the bytes transferred counter */
	MSInterfaceInfo->State.CommandBlock.DataTransferLength -= 8;

	return true;
}

/** Command processing for an issued SCSI SEND DIAGNOSTIC command. This command performs a quick check of the Dataflash ICs on the
 *  board, and indicates if they are present and functioning correctly. Only the Self-Test portion of the diagnostic command is
 *  supported.
 */
static bool SCSI_Command_Send_Diagnostic(USB_ClassInfo_MS_Device_t *const MSInterfaceInfo)
{
	/* Check to see if the SELF TEST bit is not set */
	if (!(MSInterfaceInfo->State.CommandBlock.SCSICommandData[1] & (1 << 2))) {
		/* Only self-test supported - update SENSE key and fail the command */
		SCSI_SET_SENSE(SCSI_SENSE_KEY_ILLEGAL_REQUEST,
					   SCSI_ASENSE_INVALID_FIELD_IN_CDB,
					   SCSI_ASENSEQ_NO_QUALIFIER);

		return false;
	}
	/* Succeed the command and update the bytes transferred counter */
	MSInterfaceInfo->State.CommandBlock.DataTransferLength = 0;

	return true;
}

/** Command processing for an issued SCSI READ (10) or WRITE (10) command. This command reads in the block start address
 *  and total number of blocks to process, then calls the appropriate low-level Dataflash routine to handle the actual
 *  reading and writing of the data.
 */
static bool SCSI_Command_ReadWrite_10(USB_ClassInfo_MS_Device_t *const MSInterfaceInfo,
									  const bool IsDataRead)
{
	uint32_t BlockAddress;
	uint16_t TotalBlocks;

	/* Check if the disk is write protected or not */
	if ((IsDataRead == DATA_WRITE) && DISK_READ_ONLY) {
		/* Block address is invalid, update SENSE key and return command fail */
		SCSI_SET_SENSE(SCSI_SENSE_KEY_DATA_PROTECT,
					   SCSI_ASENSE_WRITE_PROTECTED,
					   SCSI_ASENSEQ_NO_QUALIFIER);

		return false;
	}

	/* Load in the 32-bit block address (SCSI uses big-endian, so have to reverse the byte order) */
	BlockAddress =
		(MSInterfaceInfo->State.CommandBlock.SCSICommandData[2] <<
		 24) + (MSInterfaceInfo->State.CommandBlock.SCSICommandData[3] << 16) +
		(MSInterfaceInfo->State.CommandBlock.SCSICommandData[4] <<
		 8) + MSInterfaceInfo->State.CommandBlock.SCSICommandData[5];

	/* Load in the 16-bit total blocks (SCSI uses big-endian, so have to reverse the byte order) */
	TotalBlocks  =
		(MSInterfaceInfo->State.CommandBlock.SCSICommandData[7] <<
		 8) + MSInterfaceInfo->State.CommandBlock.SCSICommandData[8];

	/* Check if the block address is outside the maximum allowable value for the LUN */

	/** Under development, not working yet. */
#ifdef CFG_SDCARD
	if (BlockAddress >= MSC_Get_Block_Count())
#else
	if (BlockAddress >= LUN_MEDIA_BLOCKS)
#endif
	{
		/* Block address is invalid, update SENSE key and return command fail */
		SCSI_SET_SENSE(SCSI_SENSE_KEY_ILLEGAL_REQUEST,
					   SCSI_ASENSE_LOGICAL_BLOCK_ADDRESS_OUT_OF_RANGE,
					   SCSI_ASENSEQ_NO_QUALIFIER);

		return false;
	}

	#if (TOTAL_LUNS > 1)
	/* Adjust the given block address to the real media address based on the selected LUN */
	BlockAddress += ((uint32_t) MSInterfaceInfo->State.CommandBlock.LUN * LUN_MEDIA_BLOCKS);
	#endif

	/** Under development, not working yet. */
#ifdef CFG_SDCARD
	/* Determine if the packet is a READ (10) or WRITE (10) command, call appropriate function */
	if (IsDataRead == DATA_READ) {
		Chip_SDMMC_ReadBlocks(LPC_SDMMC, (void *) disk_cache, BlockAddress, TotalBlocks);
		while (!Endpoint_IsINReady(MSInterfaceInfo->Config.PortNumber)) {}
		Endpoint_Streaming(MSInterfaceInfo->Config.PortNumber,
						   (uint8_t *) disk_cache,
						   VIRTUAL_MEMORY_BLOCK_SIZE,
						   TotalBlocks,
						   0);
	}
	else {
		Endpoint_Streaming(MSInterfaceInfo->Config.PortNumber,
						   (uint8_t *) disk_cache,
						   VIRTUAL_MEMORY_BLOCK_SIZE,
						   TotalBlocks,
						   0);
		while (!Endpoint_IsOUTReceived(MSInterfaceInfo->Config.PortNumber)) {}
		Chip_SDMMC_WriteBlocks(LPC_SDMMC, (void *) disk_cache, BlockAddress, TotalBlocks);
	}
#else
	uint32_t startaddr;
	uint16_t blocks, dummyblocks;

	startaddr = MassStorage_GetAddressInImage(BlockAddress, TotalBlocks, &blocks);
	if (blocks == 0) {
		dummyblocks = TotalBlocks;
	}
	else if (blocks < TotalBlocks) {
		dummyblocks = TotalBlocks - blocks;
	}
	else {dummyblocks = 0; }
	Endpoint_Streaming(MSInterfaceInfo->Config.PortNumber,
					   (uint8_t *) startaddr,
					   VIRTUAL_MEMORY_BLOCK_SIZE,
					   blocks,
					   dummyblocks);
	while (!Endpoint_IsReadWriteAllowed(MSInterfaceInfo->Config.PortNumber)) {}
#endif
	/* Update the bytes transferred counter and succeed the command */
	MSInterfaceInfo->State.CommandBlock.DataTransferLength -= ((uint32_t) TotalBlocks * VIRTUAL_MEMORY_BLOCK_SIZE);

	return true;
}

/** Command processing for an issued SCSI MODE SENSE (6) command. This command returns various informational pages about
 *  the SCSI device, as well as the device's Write Protect status.
 *
 *  \param[in] MSInterfaceInfo  Pointer to the Mass Storage class interface structure that the command is associated with
 *
 *  \return Boolean true if the command completed successfully, false otherwise.
 */
static bool SCSI_Command_ModeSense_6(USB_ClassInfo_MS_Device_t *const MSInterfaceInfo)
{
	/* Send an empty header response with the Write Protect flag status */
	Endpoint_Write_8(MSInterfaceInfo->Config.PortNumber, 0x00);
	Endpoint_Write_8(MSInterfaceInfo->Config.PortNumber, 0x00);
	Endpoint_Write_8(MSInterfaceInfo->Config.PortNumber, DISK_READ_ONLY ? 0x80 : 0x00);
	Endpoint_Write_8(MSInterfaceInfo->Config.PortNumber, 0x00);
	Endpoint_ClearIN(MSInterfaceInfo->Config.PortNumber);

	/* Update the bytes transferred counter and succeed the command */
	MSInterfaceInfo->State.CommandBlock.DataTransferLength -= 4;

	return true;
}
