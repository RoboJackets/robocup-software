/*
 * @brief SCSI definitions and functions
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

#ifndef __SCSI_H_
#define __SCSI_H_

#include "board.h"
#include "DataRam.h"
#include "USB.h"
#include "../MassStorage.h"
#include "../Descriptors.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @defgroup Mass_Storage_Device_SCSI SCSI command
 * @ingroup USB_Mass_Storage_Device_18xx43xx USB_Mass_Storage_Device_17xx40xx
 * @{
 */

/** Macro to set the current SCSI sense data to the given key, additional sense code and additional sense qualifier. This
 *  is for convenience, as it allows for all three sense values (returned upon request to the host to give information about
 *  the last command failure) in a quick and easy manner.
 *
 *  @param	 Key :    New SCSI sense key to set the sense code to
 *  @param	 Acode :  New SCSI additional sense key to set the additional sense code to
 *  @param	 Aqual :  New SCSI additional sense key qualifier to set the additional sense qualifier code to
 */
#define SCSI_SET_SENSE(Key, Acode, Aqual)  MACROS {SenseData.SenseKey                 = (Key);	 \
												   SenseData.AdditionalSenseCode      = (Acode); \
												   SenseData.AdditionalSenseQualifier = (Aqual); } MACROE

/** Macro for the @ref SCSI_Command_ReadWrite_10() function, to indicate that data is to be read from the storage medium. */
#define DATA_READ           true

/** Macro for the @ref SCSI_Command_ReadWrite_10() function, to indicate that data is to be written to the storage medium. */
#define DATA_WRITE          false

/** Value for the DeviceType entry in the SCSI_Inquiry_Response_t enum, indicating a Block Media device. */
#define DEVICE_TYPE_BLOCK   0x00

/** Value for the DeviceType entry in the SCSI_Inquiry_Response_t enum, indicating a CD-ROM device. */
#define DEVICE_TYPE_CDROM   0x05

/** @brief	Main routine to process the SCSI command located in the Command Block Wrapper read from the host. This dispatches
 *          to the appropriate SCSI command handling routine if the issued command is supported by the device, else it returns
 *          a command failure due to a ILLEGAL REQUEST.
 *
 *  @param	MSInterfaceInfo :  Pointer to the Mass Storage class interface structure that the command is associated with
 *
 *  @return Boolean true if the command completed successfully, false otherwise
 */
bool SCSI_DecodeSCSICommand(USB_ClassInfo_MS_Device_t *const MSInterfaceInfo);

#if defined(INCLUDE_FROM_SCSI_C)
/** @brief	Command processing for an issued SCSI INQUIRY command. This command returns information about the device's features
 *          and capabilities to the host.
 *
 *  @param	MSInterfaceInfo :  Pointer to the Mass Storage class interface structure that the command is associated with
 *
 *  @return Boolean true if the command completed successfully, false otherwise.
 */
static bool SCSI_Command_Inquiry(USB_ClassInfo_MS_Device_t *const MSInterfaceInfo);

/** @brief	Command processing for an issued SCSI REQUEST SENSE command. This command returns information about the last issued command,
 *          including the error code and additional error information so that the host can determine why a command failed to complete.
 *
 *  @param	MSInterfaceInfo :  Pointer to the Mass Storage class interface structure that the command is associated with
 *
 *  @return Boolean true if the command completed successfully, false otherwise.
 */
static bool SCSI_Command_Request_Sense(USB_ClassInfo_MS_Device_t *const MSInterfaceInfo);

/** @brief	Command processing for an issued SCSI READ CAPACITY (10) command. This command returns information about the device's capacity
 *          on the selected Logical Unit (drive), as a number of OS-sized blocks.
 *
 *  @param	MSInterfaceInfo :  Pointer to the Mass Storage class interface structure that the command is associated with
 *
 *  @return Boolean true if the command completed successfully, false otherwise.
 */
static bool SCSI_Command_Read_Capacity_10(USB_ClassInfo_MS_Device_t *const MSInterfaceInfo);

/** @brief	Command processing for an issued SCSI SEND DIAGNOSTIC command. This command performs a quick check of the Dataflash ICs on the
 *          board, and indicates if they are present and functioning correctly. Only the Self-Test portion of the diagnostic command is
 *          supported.
 *
 *  @param	MSInterfaceInfo :  Pointer to the Mass Storage class interface structure that the command is associated with
 *
 *  @return Boolean true if the command completed successfully, false otherwise.
 */
static bool SCSI_Command_Send_Diagnostic(USB_ClassInfo_MS_Device_t *const MSInterfaceInfo);

/** @brief	Command processing for an issued SCSI READ (10) or WRITE (10) command. This command reads in the block start address
 *          and total number of blocks to process, then calls the appropriate low-level Dataflash routine to handle the actual
 *          reading and writing of the data.
 *
 *  @param  MSInterfaceInfo :  Pointer to the Mass Storage class interface structure that the command is associated with
 *  @param  IsDataRead :  Indicates if the command is a READ (10) command or WRITE (10) command (DATA_READ or DATA_WRITE)
 *
 *  @return Boolean true if the command completed successfully, false otherwise.
 */
static bool SCSI_Command_ReadWrite_10(USB_ClassInfo_MS_Device_t *const MSInterfaceInfo,
									  const bool IsDataRead);

/** @brief	Command processing for an issued SCSI MODE SENSE (6) command. This command returns various informational pages about
 *          the SCSI device, as well as the device's Write Protect status.
 *
 *  @param	MSInterfaceInfo :  Pointer to the Mass Storage class interface structure that the command is associated with
 *
 *  @return Boolean true if the command completed successfully, false otherwise.
 */
static bool SCSI_Command_ModeSense_6(USB_ClassInfo_MS_Device_t *const MSInterfaceInfo);

#endif

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* __SCSI_H_ */
