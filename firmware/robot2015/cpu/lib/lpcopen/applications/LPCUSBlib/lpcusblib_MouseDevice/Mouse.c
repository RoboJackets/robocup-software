/*
 * @brief Make your board becomes a USB mouse
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

#include "Mouse.h"

/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/

/** Buffer to hold the previously generated Mouse HID report, for comparison purposes inside the HID class driver. */
static uint8_t PrevMouseHIDReportBuffer[sizeof(USB_MouseReport_Data_t)];

/** nxpUSBlib HID Class driver interface configuration and state information. This structure is
 *  passed to all HID Class driver functions, so that multiple instances of the same class
 *  within a device can be differentiated from one another.
 */
static USB_ClassInfo_HID_Device_t Mouse_HID_Interface = {
	.Config = {
		.InterfaceNumber              = 0,

		.ReportINEndpointNumber       = MOUSE_EPNUM,
		.ReportINEndpointSize         = MOUSE_EPSIZE,
		.ReportINEndpointDoubleBank   = false,

		.PrevReportINBuffer           = PrevMouseHIDReportBuffer,
		.PrevReportINBufferSize       = sizeof(PrevMouseHIDReportBuffer),
		.PortNumber             = 0,
	},
};

/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/

/*****************************************************************************
 * Private functions
 ****************************************************************************/

/**
 * @brief Configures the board hardware and chip peripherals for the demo's functionality
 * @return Nothing
 */
static void SetupHardware(void)
{
	Board_Init();
	Board_Buttons_Init();
	Board_Joystick_Init();
	USB_Init(Mouse_HID_Interface.Config.PortNumber, USB_MODE_Device);
#if defined(USB_DEVICE_ROM_DRIVER)
	UsbdHid_Init();
#endif
}

/*****************************************************************************
 * Public functions
 ****************************************************************************/

#if defined(USB_DEVICE_ROM_DRIVER)
extern void CALLBACK_UsbdHid_SetReportChange(bool newstate);
#endif

/**
 * @brief Main program entry point
 * @return Will not return
 * @note  This routine contains the overall program flow, including initial
 * setup of all components and the main program loop
 */
int main(void)
{
	SetupHardware();

	for (;; ) {
		#if defined(USB_DEVICE_ROM_DRIVER)
		USB_MouseReport_Data_t report;
		uint16_t reportsize;
		uint8_t reportID = 0;

		memset(&report, 0, sizeof(USB_MouseReport_Data_t));
		CALLBACK_HID_Device_CreateHIDReport(&Mouse_HID_Interface, &reportID, HID_REPORT_ITEM_In, &report, &reportsize);
		if (memcmp(&report, Mouse_HID_Interface.Config.PrevReportINBuffer,
				   Mouse_HID_Interface.Config.PrevReportINBufferSize)) {
			memcpy(Mouse_HID_Interface.Config.PrevReportINBuffer,
				   &report,
				   Mouse_HID_Interface.Config.PrevReportINBufferSize);
			CALLBACK_UsbdHid_SetReportChange(true);
		}
		#else
		HID_Device_USBTask(&Mouse_HID_Interface);
		USB_USBTask(Mouse_HID_Interface.Config.PortNumber, USB_MODE_Device);
		#endif
	}
}

/**
 * @brief Event handler for the library USB Connection event
 * @return Nothing
 */
void EVENT_USB_Device_Connect(void)
{
}

/**
 * @brief Event handler for the library USB Disconnection event
 * @return Nothing
 */
void EVENT_USB_Device_Disconnect(void)
{}

/**
 * @brief Event handler for the library USB Configuration Changed event
 * @return Nothing
 */
void EVENT_USB_Device_ConfigurationChanged(void)
{
	bool ConfigSuccess = true;

	ConfigSuccess &= HID_Device_ConfigureEndpoints(&Mouse_HID_Interface);

	USB_Device_EnableSOFEvents();
}

/**
 * @brief Event handler for the library USB Control Request reception event
 * @return Nothing
 */
void EVENT_USB_Device_ControlRequest(void)
{
	HID_Device_ProcessControlRequest(&Mouse_HID_Interface);
}

/**
 * @brief Event handler for the USB device Start Of Frame event
 * @return Nothing
 */
void EVENT_USB_Device_StartOfFrame(void)
{
	HID_Device_MillisecondElapsed(&Mouse_HID_Interface);
}

/* HID class driver callback function for the creation of HID reports to the host */
bool CALLBACK_HID_Device_CreateHIDReport(USB_ClassInfo_HID_Device_t *const HIDInterfaceInfo,
										 uint8_t *const ReportID,
										 const uint8_t ReportType,
										 void *ReportData,
										 uint16_t *const ReportSize)
{
	USB_MouseReport_Data_t *MouseReport = (USB_MouseReport_Data_t *) ReportData;

	uint8_t JoyStatus_LCL    = Joystick_GetStatus();
	uint8_t ButtonStatus_LCL = Buttons_GetStatus();
	bool press = false;

	if (JoyStatus_LCL & JOY_UP) {
		MouseReport->Y = -1;
		press = true;
	}
	else if (JoyStatus_LCL & JOY_DOWN) {
		MouseReport->Y =  1;
		press = true;
	}

	if (JoyStatus_LCL & JOY_LEFT) {
		MouseReport->X = -1;
		press = true;
	}
	else if (JoyStatus_LCL & JOY_RIGHT) {
		MouseReport->X =  1;
		press = true;
	}

	if (JoyStatus_LCL & JOY_PRESS) {
		MouseReport->Button |= (1 << 0);
		press = true;
	}

	if (ButtonStatus_LCL & BUTTONS_BUTTON1) {
		MouseReport->Button |= (1 << 1);
		press = true;
	}

	*ReportSize = sizeof(USB_MouseReport_Data_t);
	return press;
}

/* HID class driver callback function for the processing of HID reports from the host */
void CALLBACK_HID_Device_ProcessHIDReport(USB_ClassInfo_HID_Device_t *const HIDInterfaceInfo,
										  const uint8_t ReportID,
										  const uint8_t ReportType,
										  const void *ReportData,
										  const uint16_t ReportSize)
{
	/* Unused (but mandatory for the HID class driver) in this demo, since there are no Host->Device reports */
}
