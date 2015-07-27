/*
 * @brief Audio device class ROM based application's specific functions supporting core layer
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

uint8_t StringDescriptor[] = {
	USB_STRING_LEN(1),	/* LanguageString */
	DTYPE_String,
	WBVAL(LANGUAGE_ID_ENG),

	USB_STRING_LEN(3),	/* ManufacturerString */
	DTYPE_String,
	WBVAL('N'), WBVAL('X'), WBVAL('P'),

	USB_STRING_LEN(18),	/* ProductString */
	DTYPE_String,
	WBVAL('L'), WBVAL('P'), WBVAL('C'), WBVAL('U'), WBVAL('S'), WBVAL('B'), WBVAL('l'), WBVAL('i'), WBVAL('b'), WBVAL(
		' '),
	WBVAL('A'), WBVAL('D'), WBVAL('C'), WBVAL(' '),
	WBVAL('D'), WBVAL('e'), WBVAL('m'), WBVAL('o')
};
extern USB_Descriptor_Device_t DeviceDescriptor;
extern USB_Descriptor_Configuration_t ConfigurationDescriptor;

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
	return 0;
}

#endif
