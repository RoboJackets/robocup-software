/*
 * @brief Generic HID Device Example
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

#include "GenericHID.h"

/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/

/** Buffer to hold the previously generated HID report, for comparison purposes inside the HID class driver. */
static uint8_t PrevHIDReportBuffer[GENERIC_REPORT_SIZE];

/** nxpUSBlib HID Class driver interface configuration and state information. This structure is
 *  passed to all HID Class driver functions, so that multiple instances of the same class
 *  within a device can be differentiated from one another.
 */
static USB_ClassInfo_HID_Device_t Generic_HID_Interface = {
	.Config = {
		.InterfaceNumber              = 0,

		.ReportINEndpointNumber       = GENERIC_IN_EPNUM,
		.ReportINEndpointSize         = GENERIC_EPSIZE,
		.ReportINEndpointDoubleBank   = false,

		.PrevReportINBuffer           = PrevHIDReportBuffer,
		.PrevReportINBufferSize       = sizeof(PrevHIDReportBuffer),
		.PortNumber             = 0,
	},
};

/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/

/*****************************************************************************
 * Private functions
 ****************************************************************************/

/* Configures the board hardware and chip peripherals for the demo's
   functionality */
static void SetupHardware(void)
{
	Board_Init();
	Board_Buttons_Init();
	Board_Joystick_Init();
	USB_Init(Generic_HID_Interface.Config.PortNumber, USB_MODE_Device);
#if defined(USB_DEVICE_ROM_DRIVER)
	UsbdHid_Init();
#endif
}

/*****************************************************************************
 * Public functions
 ****************************************************************************/

/**
 * @brief Main program entry point
 * @return Will never return
 * @note This routine contains the overall program flow, including initial
 * setup of all components and the main program loop
 */
int main(void)
{
	SetupHardware();

	for (;; ) {
		#if defined(USB_DEVICE_ROM_DRIVER)
		USB_Descriptor_HIDReport_Datatype_t report;
		uint16_t reportsize;
		uint8_t reportID = 0;

		memset(&report, 0, sizeof(USB_Descriptor_HIDReport_Datatype_t));
		CALLBACK_HID_Device_CreateHIDReport(&Generic_HID_Interface, &reportID, HID_REPORT_ITEM_In, &report, &reportsize);
		if (memcmp(&report, Generic_HID_Interface.Config.PrevReportINBuffer,
				   Generic_HID_Interface.Config.PrevReportINBufferSize)) {
			memcpy(Generic_HID_Interface.Config.PrevReportINBuffer,
				   &report,
				   Generic_HID_Interface.Config.PrevReportINBufferSize);
			CALLBACK_UsbdHid_SetReportChange(true);
		}
		#else
		HID_Device_USBTask(&Generic_HID_Interface);
		USB_USBTask(Generic_HID_Interface.Config.PortNumber, USB_MODE_Device);
		#endif
	}
}

/* Event handler for the library USB Connection event */
void EVENT_USB_Device_Connect(void)
{}

/* Event handler for the library USB Disconnection event */
void EVENT_USB_Device_Disconnect(void)
{}

/* Event handler for the library USB Configuration Changed event */
void EVENT_USB_Device_ConfigurationChanged(void)
{
	bool ConfigSuccess = true;

	ConfigSuccess &= HID_Device_ConfigureEndpoints(&Generic_HID_Interface);

	USB_Device_EnableSOFEvents();
}

/* Event handler for the library USB Control Request reception event */
void EVENT_USB_Device_ControlRequest(void)
{
	HID_Device_ProcessControlRequest(&Generic_HID_Interface);
}

/* Event handler for the USB device Start Of Frame event */
void EVENT_USB_Device_StartOfFrame(void)
{
	HID_Device_MillisecondElapsed(&Generic_HID_Interface);
}

/* HID class driver callback function for the creation of HID reports to the host */
bool CALLBACK_HID_Device_CreateHIDReport(USB_ClassInfo_HID_Device_t *const HIDInterfaceInfo,
										 uint8_t *const ReportID,
										 const uint8_t ReportType,
										 void *ReportData,
										 uint16_t *const ReportSize)
{
	uint8_t *Data = (uint8_t *) ReportData;
	uint8_t JoyStatus_LCL    = Joystick_GetStatus();
	uint8_t ButtonStatus_LCL = Buttons_GetStatus();
	uint8_t ret = 0;

	if (JoyStatus_LCL & JOY_UP) {
		ret |= 0x01;
	}
	if (JoyStatus_LCL & JOY_LEFT) {
		ret |= 0x02;
	}
	if (JoyStatus_LCL & JOY_RIGHT) {
		ret |= 0x04;
	}
	if (JoyStatus_LCL & JOY_PRESS) {
		ret |= 0x08;
	}
	if (JoyStatus_LCL & JOY_DOWN) {
		ret |= 0x10;
	}
	if (ButtonStatus_LCL & BUTTONS_BUTTON1) {
		ret |= 0x20;
	}

	Data[0] = ret;

	*ReportSize = GENERIC_REPORT_SIZE;
	return false;
}

/* HID class driver callback function for the processing of HID reports from the host */
void CALLBACK_HID_Device_ProcessHIDReport(USB_ClassInfo_HID_Device_t *const HIDInterfaceInfo,
										  const uint8_t ReportID,
										  const uint8_t ReportType,
										  const void *ReportData,
										  const uint16_t ReportSize)
{
	uint8_t *Data = (uint8_t *) ReportData;
	uint8_t  NewLEDMask = LEDS_NO_LEDS;

	if (Data[0] & 0x01) {
		NewLEDMask |= LEDS_LED1;
		Board_LED_Set(0, 1);
	}
	else {
		Board_LED_Set(0, 0);
	}
	if (Data[0] & 0x02) {
		NewLEDMask |= LEDS_LED2;
		Board_LED_Set(1, 1);
	}
	else {
		Board_LED_Set(1, 0);
	}
	if (Data[0] & 0x04) {
		NewLEDMask |= LEDS_LED3;
		Board_LED_Set(2, 1);
	}
	else {
		Board_LED_Set(2, 0);
	}
	if (Data[0] & 0x08) {
		NewLEDMask |= LEDS_LED4;
		Board_LED_Set(3, 1);
	}
	else {
		Board_LED_Set(3, 0);
	}

}
