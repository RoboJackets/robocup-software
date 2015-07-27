/*
 * @brief Virtual Serial Device and KeyBoard Host
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

#include "VirtualSerial.h"
#include "KeyboardHost.h"

/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/
 
/** nxpUSBlib HID Class driver interface configuration and state information. This structure is
 *  passed to all HID Class driver functions, so that multiple instances of the same class
 *  within a device can be differentiated from one another.
 */
static USB_ClassInfo_HID_Host_t Keyboard_HID_Interface = {
	.Config = {
		.DataINPipeNumber       = 1,
		.DataINPipeDoubleBank   = false,

		.DataOUTPipeNumber      = 2,
		.DataOUTPipeDoubleBank  = false,

		.HIDInterfaceProtocol   = HID_CSCP_KeyboardBootProtocol,
		.PortNumber = 0,
	},
};

/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/

/*****************************************************************************
 * Private functions
 ****************************************************************************/

/** Task to manage an enumerated USB keyboard once connected, to display key state
 *  data as it is received.
 */
static void KeyboardHost_Task(void)
{
	if (USB_HostState[Keyboard_HID_Interface.Config.PortNumber] != HOST_STATE_Configured) {
		return;
	}

	if (HID_Host_IsReportReceived(&Keyboard_HID_Interface)) {
		USB_KeyboardReport_Data_t KeyboardReport;
		HID_Host_ReceiveReport(&Keyboard_HID_Interface, &KeyboardReport);

		uint8_t KeyCode = KeyboardReport.KeyCode[0];

		if (KeyCode) {
			char PressedKey = 0;

			/* Retrieve pressed key character if alphanumeric */
			if ((KeyCode >= HID_KEYBOARD_SC_A) && (KeyCode <= HID_KEYBOARD_SC_Z)) {
				PressedKey = (KeyCode - HID_KEYBOARD_SC_A) + 'A';
			}
			else if ((KeyCode >= HID_KEYBOARD_SC_1_AND_EXCLAMATION) &
					 (KeyCode  < HID_KEYBOARD_SC_0_AND_CLOSING_PARENTHESIS)) {
				PressedKey = (KeyCode - HID_KEYBOARD_SC_1_AND_EXCLAMATION) + '1';
			}
			else if (KeyCode == HID_KEYBOARD_SC_0_AND_CLOSING_PARENTHESIS) {
				PressedKey = '0';
			}
			else if (KeyCode == HID_KEYBOARD_SC_SPACE) {
				PressedKey = ' ';
			}
			else if (KeyCode == HID_KEYBOARD_SC_ENTER) {
				PressedKey = '\n';
			}

			if (PressedKey) {
				VirtualSerial_putchar(PressedKey);
			}
		}
	}
}

/*****************************************************************************
 * Public functions
 ****************************************************************************/

/** Main program entry point. This routine configures the hardware required by the application, then
 *  enters a loop to run the application tasks in sequence.
 */
void KeyboardHost(void)
{
	static int initialized = 0;

	/* Initialize the keyboard host driver once */
	if (!initialized) {
		USB_Init(Keyboard_HID_Interface.Config.PortNumber, USB_MODE_Host);
		initialized = 1;
	}

	KeyboardHost_Task();

	HID_Host_USBTask(&Keyboard_HID_Interface);
	USB_USBTask(Keyboard_HID_Interface.Config.PortNumber, USB_MODE_Host);
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
	Keyboard_HID_Interface.Config.PortNumber = corenum;

	if (HID_Host_ConfigurePipes(&Keyboard_HID_Interface,
								ConfigDescriptorSize, ConfigDescriptorData) != HID_ENUMERROR_NoError) {
		DEBUGOUT("Attached Device Not a Valid Keyboard.\r\n");
		return;
	}

	if (USB_Host_SetDeviceConfiguration(Keyboard_HID_Interface.Config.PortNumber, 1) != HOST_SENDCONTROL_Successful) {
		DEBUGOUT("Error Setting Device Configuration.\r\n");
		return;
	}

	if (HID_Host_SetBootProtocol(&Keyboard_HID_Interface) != 0) {
		DEBUGOUT("Could not Set Boot Protocol Mode.\r\n");
		USB_Host_SetDeviceConfiguration(Keyboard_HID_Interface.Config.PortNumber, 0);
		return;
	}

	DEBUGOUT("Keyboard Enumerated.\r\n");
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

/** Dummy callback function for HID Parser
 */
bool CALLBACK_HIDParser_FilterHIDReportItem(HID_ReportItem_t *const CurrentItem)
{
	return true;
}
