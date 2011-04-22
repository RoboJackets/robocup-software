#include <usb/device/cdc-serial/CDCDSerialDriver.h>
#include <board.h>

#include "status.h"
#include "control.h"
#include "console.h"

unsigned int robot_id;
unsigned int failures;
uint8_t motor_faults;
uint8_t kicker_status;

// Detects and handles USB connection/disconnection
enum
{
	USB_Disconnected,
	USB_Connected,
	USB_Working
} usb_state = USB_Disconnected;

void check_usb_connection()
{
	int vbus = AT91C_BASE_PIOA->PIO_PDSR & VBUS;
	if (vbus && usb_state == USB_Disconnected)
	{
		// Initialize the USB controller.
		// This will disconnect from the host (important if we have just reset after reprogramming).
		// The host will notice disconnection while we check the FPGA below.
		CDCDSerialDriver_Initialize();
		USBD_Connect();
		usb_state = USB_Connected;
	}
	if (!vbus)
	{
		usb_state = USB_Disconnected;
	}
	
	if (usb_state == USB_Connected && USBD_GetState() >= USBD_STATE_CONFIGURED)
	{
		console_init();
		usb_state = USB_Working;
	}
	
	if (usb_state == USB_Working)
	{
		if (!console_run())
		{
			controller = 0;
		}
	}
}
