/*
 * @brief Generic HID device class ROM based application's specific functions supporting core layer
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

#define  __INCLUDE_FROM_USB_DRIVER
#include "..\GenericHID.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @defgroup Generic_HID_Device_ROM_Based ROM stack support
 * @ingroup Generic_HID_Device_18xx43xx Generic_HID_Device_11Uxx
 * @ingroup Generic_HID_Device
 * @{
 */

/** @defgroup Generic_HID_Device_ROM_Based_Core ROM based core
 * @{
 */

/**
 * @brief	Call back function registers device descriptor array's address to ROM stack
 * @return	Address of device descriptor array
 */
uint32_t CALLBACK_UsbdRom_Register_DeviceDescriptor(void);

/**
 * @brief	Call back function registers device configuration descriptor array's address to ROM stack
 * @return	Address of device configuration descriptor array
 */
uint32_t CALLBACK_UsbdRom_Register_ConfigurationDescriptor(void);

/**
 * @brief	Call back function registers string descriptor array's address to ROM stack
 * @return	Address of string descriptor array
 */
uint32_t CALLBACK_UsbdRom_Register_StringDescriptor(void);

/**
 * @brief	Call back function registers device qualifier descriptor array's address to ROM stack
 * @return	Address of device qualifier descriptor array
 */
uint32_t CALLBACK_UsbdRom_Register_DeviceQualifierDescriptor(void);

/**
 * @brief	Call back function registers endpoint configuration descriptor array's address to ROM stack
 * @return	Address of endpoint configuration descriptor array
 */
uint8_t CALLBACK_UsbdRom_Register_ConfigureEndpoint(void);

/**
 * @}
 */

/** @defgroup Generic_HID_Device_ROM_Based_HID_Device_Class ROM based class HID
 * @{
 */

/**
 * @brief	Call back function registers interface descriptor
 * @return	Address of interface descriptor array
 */
uint32_t CALLBACK_UsbdHid_Register_InterfaceDescriptor(void);

/**
 * @brief	Call back function registers report descriptor
 * @param	dest	: Pointer to destination
 * @return	Length report
 */
uint32_t CALLBACK_UsbdHid_Register_ReportDescriptor(uint8_t * *dest);

/**
 * @brief	Call back function registers report In buffer
 * @param	dest	: Pointer to destination
 * @return	Size of In buffer
 */
uint32_t CALLBACK_UsbdHid_Register_ReportInBuffer(uint8_t * *dest);

/**
 * @brief	Call back function set report
 * @param	reportoutbuffer	: Pointer to out buffer
 * @param	reportoutsize	: Size of out report
 * @return	Nothing
 */
void CALLBACK_UsbdHid_SetReport(uint8_t * *reportoutbuffer, uint32_t reportoutsize);

/**
 * @brief	Call back function set report change
 * @param	newstate	: next state
 * @return	Nothing
 */
void CALLBACK_UsbdHid_SetReportChange(bool newstate);

/**
 * @brief	Determine if report has been changed
 * @return	Boolean
 */
bool CALLBACK_UsbdHid_IsReportChanged(void);

/**
 * @}
 */

/**
 * @}
 */

#ifdef __cplusplus
}
#endif
