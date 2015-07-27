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
 
#define ECHO_CHARACTER_TASK     (0)
#define CDC_BRIDGE_TASK         (ECHO_CHARACTER_TASK + 1)

/** LPCUSBlib CDC Class driver interface configuration and state information. This structure is
 *  passed to all CDC Class driver functions, so that multiple instances of the same class
 *  within a device can be differentiated from one another.
 */
static USB_ClassInfo_CDC_Device_t VirtualSerial_CDC_Interface = {
	.Config = {
		.ControlInterfaceNumber         = 0,

		.DataINEndpointNumber           = CDC_TX_EPNUM,
		.DataINEndpointSize             = CDC_TXRX_EPSIZE,
		.DataINEndpointDoubleBank       = false,

		.DataOUTEndpointNumber          = CDC_RX_EPNUM,
		.DataOUTEndpointSize            = CDC_TXRX_EPSIZE,
		.DataOUTEndpointDoubleBank      = false,

		.NotificationEndpointNumber     = CDC_NOTIFICATION_EPNUM,
		.NotificationEndpointSize       = CDC_NOTIFICATION_EPSIZE,
		.NotificationEndpointDoubleBank = false,
		.PortNumber = 1,
	},
};

/** Select example task, currently lpc11Uxx and lpc17xx don't support for bridging task
 * Only LPC18xx has this feature */
#define CDC_TASK_SELECT ECHO_CHARACTER_TASK

/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/

/*****************************************************************************
 * Private functions
 ****************************************************************************/

/** Configures the board hardware and chip peripherals for the demo's functionality. */
static void SetupHardware(void)
{
	Board_Init();
	Board_Debug_Init();
	USB_Init(VirtualSerial_CDC_Interface.Config.PortNumber, USB_MODE_Device);
	Serial_CreateStream(NULL);
}

#if (CDC_TASK_SELECT == ECHO_CHARACTER_TASK)
/** Checks for data input, reply back to the host. */
static char EchoCharacter(void)
{
	/* Echo back character */
	uint8_t recv_byte[CDC_TXRX_EPSIZE];

	if (CDC_Device_BytesReceived(&VirtualSerial_CDC_Interface)) {
		recv_byte[0] = CDC_Device_ReceiveByte(&VirtualSerial_CDC_Interface);
		CDC_Device_SendData(&VirtualSerial_CDC_Interface, (char *) recv_byte, 1);
	}

	return recv_byte[0];
}

#else
/** USB-UART Bridge Task */
static void CDC_Bridge_Task(void)
{
	/* Echo back character */
	uint8_t out_buff[CDC_TXRX_EPSIZE], in_buff[CDC_TXRX_EPSIZE];
	uint32_t recv_count;

	recv_count = CDC_Device_BytesReceived(&VirtualSerial_CDC_Interface);
	while (recv_count--) {
		out_buff[0] = CDC_Device_ReceiveByte(&VirtualSerial_CDC_Interface);
		Serial_Send((uint8_t *) out_buff, 1, BLOCKING);
	}

	recv_count = Serial_Revc(in_buff, CDC_TXRX_EPSIZE, NONE_BLOCKING);
	if (recv_count) {
		CDC_Device_SendData(&VirtualSerial_CDC_Interface, (char *) in_buff, recv_count);
	}
}

#endif

/*****************************************************************************
 * Public functions
 ****************************************************************************/

/** Main program entry point. This routine contains the overall program flow, including initial
 *  setup of all components and the main program loop.
 */
int main(void)
{
	SetupHardware();
	DEBUGOUT("Keyboard Host Demo running.\r\n");
	for (;; ) {
		EchoCharacter();
		USB_USBTask(VirtualSerial_CDC_Interface.Config.PortNumber, USB_MODE_Device);
		KeyboardHost();
	}
}

/* Virtual serial putchar */
void VirtualSerial_putchar(int ch)
{
	char tmp = ch;
	CDC_Device_SendData(&VirtualSerial_CDC_Interface, &tmp, 1);
}

/** Event handler for the library USB Connection event. */
void EVENT_USB_Device_Connect(void)
{}

/** Event handler for the library USB Disconnection event. */
void EVENT_USB_Device_Disconnect(void)
{}

/** Event handler for the library USB Configuration Changed event. */
void EVENT_USB_Device_ConfigurationChanged(void)
{
	bool ConfigSuccess = true;

	ConfigSuccess &= CDC_Device_ConfigureEndpoints(&VirtualSerial_CDC_Interface);

}

/** Event handler for the library USB Control Request reception event. */
void EVENT_USB_Device_ControlRequest(void)
{
	CDC_Device_ProcessControlRequest(&VirtualSerial_CDC_Interface);
}

void EVENT_CDC_Device_LineEncodingChanged(USB_ClassInfo_CDC_Device_t *const CDCInterfaceInfo)
{
	/* TODO: add LineEncoding processing here
	 * this is just a simple statement, only Baud rate is set */
	/* Serial_Init(CDCInterfaceInfo->State.LineEncoding.BaudRateBPS, false); */
}

#if defined(USB_DEVICE_ROM_DRIVER)
	#error This demo is not supposed to run under ROM STACK mode
#endif
