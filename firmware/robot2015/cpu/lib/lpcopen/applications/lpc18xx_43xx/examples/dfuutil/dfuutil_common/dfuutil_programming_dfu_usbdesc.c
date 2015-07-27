/*
 * @brief DFU Utility USB descriptors
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

#include "board.h"
#include "app_usbd_cfg.h"
#include "usbd_desc.h"
#include "usbd.h"
#include "usbd_core.h"
#include "usbd_msc.h"
#include "usbd_dfu.h"

/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/

/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/

/**
 * USB Standard Device Descriptor
 */
ALIGNED(4) uint8_t USB_DeviceDescriptor[] = {
	USB_DEVICE_DESC_SIZE,			/* bLength */
	USB_DEVICE_DESCRIPTOR_TYPE,		/* bDescriptorType */
	WBVAL(0x0200),	/* 2.00 */          /* bcdUSB */
	0x00,							/* bDeviceClass */
	0x00,							/* bDeviceSubClass */
	0x00,							/* bDeviceProtocol */
	USB_MAX_PACKET0,				/* bMaxPacketSize0 */
	WBVAL(0x1FC9),					/* idVendor */
	WBVAL(0x000C),					/* idProduct */
	WBVAL(0x0100),	/* 1.00 */          /* bcdDevice */
	0x01,							/* iManufacturer */
	0x02,							/* iProduct */
	0x03,							/* iSerialNumber */
	0x01							/* bNumConfigurations */
};

/**
 * USB FSConfiguration Descriptor
 * All Descriptors (Configuration, Interface, Endpoint, Class, Vendor)
 */
ALIGNED(4) uint8_t USB_FsConfigDescriptor[] = {
	/* Configuration 1 */
	USB_CONFIGUARTION_DESC_SIZE,	/* bLength */
	USB_CONFIGURATION_DESCRIPTOR_TYPE,	/* bDescriptorType */
	WBVAL(							/* wTotalLength */
		1 * USB_CONFIGUARTION_DESC_SIZE +
		1 * USB_INTERFACE_DESC_SIZE     +
		DFU_FUNC_DESC_SIZE
		),
	0x01,							/* bNumInterfaces */
	0x01,							/* bConfigurationValue */
	0x00,							/* iConfiguration */
	USB_CONFIG_SELF_POWERED,		/* bmAttributes */
	USB_CONFIG_POWER_MA(100),		/* bMaxPower */
	/* Interface 0, Alternate Setting 0, DFU Class */
	USB_INTERFACE_DESC_SIZE,		/* bLength */
	USB_INTERFACE_DESCRIPTOR_TYPE,	/* bDescriptorType */
	0x00,							/* bInterfaceNumber */
	0x00,							/* bAlternateSetting */
	0x00,							/* bNumEndpoints */
	USB_DEVICE_CLASS_APP,			/* bInterfaceClass */
	USB_DFU_SUBCLASS,				/* bInterfaceSubClass */
	0x01,								/* bInterfaceProtocol */
	0x04,							/* iInterface */
	/* DFU RunTime/DFU Mode Functional Descriptor */
	DFU_FUNC_DESC_SIZE,				/* bLength */
	USB_DFU_DESCRIPTOR_TYPE,		/* bDescriptorType */
	USB_DFU_CAN_DOWNLOAD | USB_DFU_CAN_UPLOAD | USB_DFU_MANIFEST_TOL,	// | USB_DFU_WILL_DETACH, /* bmAttributes */
	WBVAL(0xFF00),					/* wDetachTimeout */
	WBVAL(USB_DFU_XFER_SIZE),		/* wTransferSize */
	WBVAL(0x100),					/* bcdDFUVersion */
	/* Terminator */
	0								/* bLength */
};

/**
 * USB HSConfiguration Descriptor
 * All Descriptors (Configuration, Interface, Endpoint, Class, Vendor)
 */
ALIGNED(4) uint8_t USB_HsConfigDescriptor[] = {
	/* Configuration 1 */
	USB_CONFIGUARTION_DESC_SIZE,	/* bLength */
	USB_CONFIGURATION_DESCRIPTOR_TYPE,	/* bDescriptorType */
	WBVAL(							/* wTotalLength */
		1 * USB_CONFIGUARTION_DESC_SIZE +
		1 * USB_INTERFACE_DESC_SIZE     +
		DFU_FUNC_DESC_SIZE
		),
	0x01,							/* bNumInterfaces */
	0x01,							/* bConfigurationValue */
	0x00,							/* iConfiguration */
	USB_CONFIG_SELF_POWERED,	/* bmAttributes */
	USB_CONFIG_POWER_MA(100),		/* bMaxPower */
	/* Interface 0, Alternate Setting 0, DFU Class */
	USB_INTERFACE_DESC_SIZE,		/* bLength */
	USB_INTERFACE_DESCRIPTOR_TYPE,	/* bDescriptorType */
	0x00,							/* bInterfaceNumber */
	0x00,							/* bAlternateSetting */
	0x00,							/* bNumEndpoints */
	USB_DEVICE_CLASS_APP,			/* bInterfaceClass */
	USB_DFU_SUBCLASS,				/* bInterfaceSubClass */
	0x01,							/* bInterfaceProtocol */
	0x04,							/* iInterface */
	/* DFU RunTime/DFU Mode Functional Descriptor */
	DFU_FUNC_DESC_SIZE,				/* bLength */
	USB_DFU_DESCRIPTOR_TYPE,		/* bDescriptorType */
	USB_DFU_CAN_DOWNLOAD | USB_DFU_CAN_UPLOAD | USB_DFU_MANIFEST_TOL,	//  | USB_DFU_WILL_DETACH, /* bmAttributes */
	WBVAL(0xFF00),					/* wDetachTimeout */
	WBVAL(USB_DFU_XFER_SIZE),		/* wTransferSize */
	WBVAL(0x100),					/* bcdDFUVersion */
	/* Terminator */
	0								/* bLength */
};

/**
 * USB String Descriptor (optional)
 */
ALIGNED(4) uint8_t USB_StringDescriptor[] = {
	/* Index 0x00: LANGID Codes */
	0x04,							/* bLength */
	USB_STRING_DESCRIPTOR_TYPE,		/* bDescriptorType */
	WBVAL(0x0409),	/* US English */    /* wLANGID */
	/* Index 0x01: Manufacturer */
	(3 * 2 + 2),					/* bLength (13 Char + Type + lenght) */
	USB_STRING_DESCRIPTOR_TYPE,		/* bDescriptorType */
	'N', 0,
	'X', 0,
	'P', 0,
	/* Index 0x02: Product */
	(3 * 2 + 2),					/* bLength (13 Char + Type + lenght) */
	USB_STRING_DESCRIPTOR_TYPE,		/* bDescriptorType */
	'L', 0,
	'P', 0,
	'C', 0,
	/* Index 0x03: Serial Number */
	(4 * 2 + 2),					/* bLength (13 Char + Type + lenght) */
	USB_STRING_DESCRIPTOR_TYPE,		/* bDescriptorType */
	'4', 0,
	'3', 0,
	'5', 0,
	'0', 0,
	/* Index 0x04: Interface 0, Alternate Setting 0 */
	(3 * 2 + 2),					/* bLength (3 Char + Type + lenght) */
	USB_STRING_DESCRIPTOR_TYPE,		/* bDescriptorType */
	'D', 0,
	'F', 0,
	'U', 0,
	/* Index 0x05: Interface 1, Alternate Setting 0 */
};

/**
 * USB Device Qualifier
 */
ALIGNED(4) uint8_t USB_DeviceQualifier[] = {
	USB_DEVICE_QUALI_SIZE,				/* bLength */
	USB_DEVICE_QUALIFIER_DESCRIPTOR_TYPE,	/* bDescriptorType */
	WBVAL(0x0200),	/* 2.00 */          /* bcdUSB */
	0x00,							/* bDeviceClass */
	0x00,							/* bDeviceSubClass */
	0x00,							/* bDeviceProtocol */
	USB_MAX_PACKET0,				/* bMaxPacketSize0 */
	0x01,							/* bNumOtherSpeedConfigurations */
	0x00							/* bReserved */
};

/*****************************************************************************
 * Private functions
 ****************************************************************************/

/*****************************************************************************
 * Public functions
 ****************************************************************************/
