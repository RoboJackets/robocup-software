/*
 * @brief Keyboard HID device class ROM based application's specific functions supporting core layer
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

#include "UsbRom.h"

#if defined(USB_DEVICE_ROM_DRIVER)

/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/

/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/

uint8_t StringDescriptor[] = {
	USB_STRING_LEN(1),	/* LanguageString */
	DTYPE_String,
	WBVAL(LANGUAGE_ID_ENG),

	USB_STRING_LEN(3),	/* ManufacturerString */
	DTYPE_String,
	WBVAL('N'), WBVAL('X'), WBVAL('P'),

	USB_STRING_LEN(22 + 1),	/* ProductString */
	DTYPE_String,
	WBVAL('L'), WBVAL('P'), WBVAL('C'), WBVAL('U'), WBVAL('S'), WBVAL('B'), WBVAL('l'), WBVAL('i'), WBVAL('b'), WBVAL(
		' '),
	WBVAL('K'), WBVAL('e'), WBVAL('y'), WBVAL('b'), WBVAL('o'), WBVAL('a'), WBVAL('r'), WBVAL('d'), WBVAL(' '),
	WBVAL('D'), WBVAL('e'), WBVAL('m'), WBVAL('o')
};
extern USB_Descriptor_Device_t DeviceDescriptor;
extern USB_Descriptor_Configuration_t ConfigurationDescriptor;
extern USB_Descriptor_HIDReport_Datatype_t KeyboardReport[];
extern USB_ClassInfo_HID_Device_t Keyboard_HID_Interface;
static bool reportchange;

/*****************************************************************************
 * Private functions
 ****************************************************************************/

/*****************************************************************************
 * Public functions
 ****************************************************************************/

uint32_t CALLBACK_UsbdRom_Register_DeviceDescriptor(void)
{
	return (uint32_t) &DeviceDescriptor;
}

uint32_t CALLBACK_UsbdRom_Register_ConfigurationDescriptor(void)
{
	return (uint32_t) &ConfigurationDescriptor;
}

uint32_t CALLBACK_UsbdRom_Register_StringDescriptor(void)
{
	return (uint32_t) StringDescriptor;
}

uint32_t CALLBACK_UsbdRom_Register_DeviceQualifierDescriptor(void)
{
	return (uint32_t) NULL;
}

uint8_t CALLBACK_UsbdRom_Register_ConfigureEndpoint(void)
{
	return Keyboard_HID_Interface.Config.ReportINEndpointNumber;
}

uint32_t CALLBACK_UsbdHid_Register_InterfaceDescriptor(void)
{
	return (uint32_t) &(ConfigurationDescriptor.HID_Interface);
}

uint32_t CALLBACK_UsbdHid_Register_ReportDescriptor(uint8_t * *dest)
{
	*dest = (uint8_t *) KeyboardReport;
	return ConfigurationDescriptor.HID_KeyboardHID.HIDReportLength;
}

uint32_t CALLBACK_UsbdHid_Register_ReportInBuffer(uint8_t * *dest)
{
	*dest = (uint8_t *) Keyboard_HID_Interface.Config.PrevReportINBuffer;
	return (uint32_t) Keyboard_HID_Interface.Config.PrevReportINBufferSize;
}

void CALLBACK_UsbdHid_SetReport(uint8_t * *reportoutbuffer, uint32_t reportoutsize)
{}

void CALLBACK_UsbdHid_SetReportChange(bool newstate)
{
	reportchange = newstate;
}

bool CALLBACK_UsbdHid_IsReportChanged(void)
{
	return reportchange;
}

#endif
