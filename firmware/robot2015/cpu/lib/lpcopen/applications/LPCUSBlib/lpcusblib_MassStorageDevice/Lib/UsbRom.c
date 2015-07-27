/*
 * @brief Mass Storage device class ROM based application's specific functions supporting core layer
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

typedef struct _st_Inquiry_String {
	uint8_t  VendorID[8];
	uint8_t  ProductID[16];
	uint8_t  RevisionID[4];
} ATTR_PACKED InquiryString;

InquiryString InquiryData = {
	.VendorID            = "NXP",
	.ProductID           = "Dataflash Disk",
	.RevisionID          = {'0', '.', '0', '0'},
};

USB_Descriptor_DeviceQualifier_t DeviceQualifierDescriptor = {
	.Header = {.Size = sizeof(USB_Descriptor_DeviceQualifier_t), .Type = DTYPE_DeviceQualifier},

	.USBSpecification       = VERSION_BCD(02.00),
	.Class                  = USB_CSCP_NoDeviceClass,
	.SubClass               = USB_CSCP_NoDeviceSubclass,
	.Protocol               = USB_CSCP_NoDeviceProtocol,

	.Endpoint0Size          = FIXED_CONTROL_ENDPOINT_SIZE,
	.NumberOfConfigurations = FIXED_NUM_CONFIGURATIONS
};

uint8_t StringDescriptor[] = {
	USB_STRING_LEN(1),	/* LanguageString */
	DTYPE_String,
	WBVAL(LANGUAGE_ID_ENG),

	USB_STRING_LEN(3),	/* ManufacturerString */
	DTYPE_String,
	WBVAL('N'), WBVAL('X'), WBVAL('P'),

	USB_STRING_LEN(22 + 5),	/* ProductString */
	DTYPE_String,
	WBVAL('L'), WBVAL('P'), WBVAL('C'), WBVAL('U'), WBVAL('S'), WBVAL('B'), WBVAL('l'), WBVAL('i'), WBVAL('b'), WBVAL(
		' '),
	WBVAL('M'), WBVAL('a'), WBVAL('s'), WBVAL('s'), WBVAL(' '), WBVAL('S'), WBVAL('t'), WBVAL('o'), WBVAL('r'), WBVAL(
		'a'), WBVAL('g'), WBVAL('e'), WBVAL(' '),
	WBVAL('D'), WBVAL('e'), WBVAL('m'), WBVAL('o')
};
extern USB_Descriptor_Device_t DeviceDescriptor;
extern USB_Descriptor_Configuration_t ConfigurationDescriptor;

extern uint8_t DiskImage[];

/*****************************************************************************
 * Private functions
 ****************************************************************************/

/*****************************************************************************
 * Public functions
 ****************************************************************************/

void translate_rd(uint32_t offset, uint8_t * *buff_adr, uint32_t length);

void translate_rd(uint32_t offset, uint8_t * *buff_adr, uint32_t length)
{
	*buff_adr =  &DiskImage[offset];
}

void translate_wr(uint32_t offset, uint8_t * *buff_adr, uint32_t length);

void translate_wr(uint32_t offset, uint8_t * *buff_adr, uint32_t length)
{
	*buff_adr =  &DiskImage[offset + length];
}

void translate_GetWrBuf(uint32_t offset, uint8_t * *buff_adr, uint32_t length);

void translate_GetWrBuf(uint32_t offset, uint8_t * *buff_adr, uint32_t length)
{
	*buff_adr =  &DiskImage[offset];
}

ErrorCode_t translate_verify(uint32_t offset, uint8_t *src, uint32_t length);

ErrorCode_t translate_verify(uint32_t offset, uint8_t *src, uint32_t length)
{
	if (memcmp(&DiskImage[offset], src, length)) {
		return ERR_FAILED;
	}

	return LPC_OK;

}

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
	return (uint32_t) &DeviceQualifierDescriptor;
}

uint8_t CALLBACK_UsbdRom_Register_ConfigureEndpoint(void)
{
	return 0;
}

uint32_t CALLBACK_UsbdMsc_Register_InquiryData(void)
{
	return (uint32_t) &InquiryData;
}

uint32_t CALLBACK_UsbdMsc_Register_BlockCount(void)
{
	return VIRTUAL_MEMORY_BLOCKS;
}

uint32_t CALLBACK_UsbdMsc_Register_BlockSize(void)
{
	return VIRTUAL_MEMORY_BLOCK_SIZE;
}

uint32_t CALLBACK_UsbdMsc_Register_MemorySize(void)
{
	return VIRTUAL_MEMORY_BYTES;
}

uint32_t CALLBACK_UsbdMsc_Register_InterfaceDescriptor(void)
{
	return (uint32_t) &(ConfigurationDescriptor.MS_Interface);
}

uint32_t CALLBACK_UsbdMsc_Register_MSCWrite(void)
{
	return (uint32_t) translate_wr;
}

uint32_t CALLBACK_UsbdMsc_Register_MSCRead(void)
{
	return (uint32_t) translate_rd;
}

uint32_t CALLBACK_UsbdMsc_Register_MSCVerify(void)
{
	return (uint32_t) translate_verify;
}

uint32_t CALLBACK_UsbdMsc_Register_MSCGetWriteBuf(void)
{
	return (uint32_t) translate_GetWrBuf;
}

#endif
