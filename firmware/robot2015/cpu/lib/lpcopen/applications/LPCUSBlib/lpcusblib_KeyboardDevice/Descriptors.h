/*
 * @brief Keyboard HID device class declarations, definitions for using in application
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

#ifndef __DESCRIPTORS_H_
#define __DESCRIPTORS_H_

#include "USB.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @defgroup Keyboard_Device_Descriptor Class descriptors
 * @ingroup USB_Keyboard_Device_18xx43xx USB_Keyboard_Device_17xx40xx USB_Keyboard_Device_11Uxx
 * @{
 */

/** @brief	Type define for the device configuration descriptor structure. This must be defined in the
 *          application code, as the configuration descriptor contains several sub-descriptors which
 *          vary between devices, and which describe the device's usage to the host.
 */
typedef struct {
	USB_Descriptor_Configuration_Header_t Config;

	// Keyboard HID Interface
	USB_Descriptor_Interface_t            HID_Interface;
	USB_HID_Descriptor_HID_t              HID_KeyboardHID;
	USB_Descriptor_Endpoint_t             HID_ReportINEndpoint;
	unsigned char                         HID_Termination;
} USB_Descriptor_Configuration_t;

/** Endpoint number of the Keyboard HID reporting IN endpoint. */
#define KEYBOARD_EPNUM               1

/** Size in bytes of the Keyboard HID reporting IN endpoint. */
#define KEYBOARD_EPSIZE              8

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* __DESCRIPTORS_H_ */
