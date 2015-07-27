/*
 * @brief Make your board becomes a USB microphone
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

#include "AudioInput.h"

/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/

/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/

/** Audio Class driver interface configuration and state information. This structure is
 *  passed to all Audio Class driver functions, so that multiple instances of the same class
 *  within a device can be differentiated from one another.
 */
USB_ClassInfo_Audio_Device_t Microphone_Audio_Interface = {
	.Config = {
		.StreamingInterfaceNumber = 1,

		.DataINEndpointNumber     = AUDIO_STREAM_EPNUM,
		.DataINEndpointSize       = AUDIO_STREAM_EPSIZE,
		.PortNumber             = 0,
	},
};

/** Max Sample Frequency. */
#define AUDIO_MAX_SAMPLE_FREQ   48000
/** Current audio sampling frequency of the streaming audio endpoint. */
uint32_t CurrentAudioSampleFrequency = AUDIO_MAX_SAMPLE_FREQ;
/* Sample Buffer */
// uint16_t* sample_buffer = NULL;
PRAGMA_ALIGN_4
uint16_t sample_buffer[512] ATTR_ALIGNED(4) __BSS(USBRAM_SECTION);
uint32_t sample_buffer_size = 0;

/*****************************************************************************
 * Private functions
 ****************************************************************************/

/* Configures the board hardware and chip peripherals for the demo's
   functionality. */
static void SetupHardware(void)
{
	Board_Init();
	Board_Buttons_Init();
	USB_Init(Microphone_Audio_Interface.Config.PortNumber, USB_MODE_Device);
}

/*****************************************************************************
 * Public functions
 ****************************************************************************/

void Audio_Reset_Data_Buffer(void)
{}

void Audio_Init(uint32_t samplefreq)
{
	CurrentAudioSampleFrequency = samplefreq;
	sample_buffer_size = samplefreq * sizeof(uint16_t) / 1000;
}

/** This callback function provides iso buffer address for HAL iso transfer processing.
 * for ISO In EP, this function also returns the size of buffer, depend on SampleFrequency.
 */
uint32_t CALLBACK_HAL_GetISOBufferAddress(const uint32_t EPNum, uint32_t *packet_size) {

	/* Check if this is audio stream endpoint */
	*packet_size = sample_buffer_size;
	if ((EPNum & 0x7F) == AUDIO_STREAM_EPNUM) {
		return (uint32_t) &sample_buffer[0];
	}
	else {return 0; }
}

/** Main program entry point. This routine contains the overall program flow, including initial
 *  setup of all components and the main program loop.
 */
int main(void)
{
	uint32_t Button_State = 0;
	SetupHardware();

	// sample_buffer = (uint16_t*)Audio_Get_ISO_Buffer_Address(0);
#if defined(USB_DEVICE_ROM_DRIVER)
	UsbdAdc_Init(&Microphone_Audio_Interface);
#endif

	for (;; ) {
		/* Only generate audio if the board button is being pressed */
		/* Generate Square Wave at 1kHz */
		if ((Buttons_GetStatus() & BUTTONS_BUTTON1) != Button_State) {
			int i;
			Button_State ^= BUTTONS_BUTTON1;
			for (i = 0; i < sample_buffer_size / 4; i++) {
				sample_buffer[i] = (Button_State << 15);
			}
		}
#if !defined(USB_DEVICE_ROM_DRIVER)
		Audio_Device_USBTask(&Microphone_Audio_Interface);
		USB_USBTask(Microphone_Audio_Interface.Config.PortNumber, USB_MODE_Device);
#endif
	}
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

	ConfigSuccess &= Audio_Device_ConfigureEndpoints(&Microphone_Audio_Interface);

	//	LEDs_SetAllLEDs(ConfigSuccess ? LEDMASK_USB_READY : LEDMASK_USB_ERROR);
}

/** Event handler for the library USB Control Request reception event. */
void EVENT_USB_Device_ControlRequest(void)
{
	Audio_Device_ProcessControlRequest(&Microphone_Audio_Interface);
}

/** Audio class driver callback for the setting and retrieval of streaming endpoint properties. This callback must be implemented
 *  in the user application to handle property manipulations on streaming audio endpoints.
 */
bool CALLBACK_Audio_Device_GetSetEndpointProperty(USB_ClassInfo_Audio_Device_t *const AudioInterfaceInfo,
												  const uint8_t EndpointProperty,
												  const uint8_t EndpointAddress,
												  const uint8_t EndpointControl,
												  uint16_t *const DataLength,
												  uint8_t *Data)
{
	/* Check the requested endpoint to see if a supported endpoint is being manipulated */
	if (EndpointAddress == (ENDPOINT_DIR_IN | Microphone_Audio_Interface.Config.DataINEndpointNumber)) {
		/* Check the requested control to see if a supported control is being manipulated */
		if (EndpointControl == AUDIO_EPCONTROL_SamplingFreq) {
			switch (EndpointProperty) {
			case AUDIO_REQ_SetCurrent:
				/* Check if we are just testing for a valid property, or actually adjusting it */
				if (DataLength != NULL) {
					/* Set the new sampling frequency to the value given by the host */
					CurrentAudioSampleFrequency =
						(((uint32_t) Data[2] << 16) | ((uint32_t) Data[1] << 8) | (uint32_t) Data[0]);
					if (CurrentAudioSampleFrequency > AUDIO_MAX_SAMPLE_FREQ) {
						return false;
					}
					sample_buffer_size = CurrentAudioSampleFrequency * sizeof(uint16_t) / 1000;
				}

				return true;

			case AUDIO_REQ_GetCurrent:
				/* Check if we are just testing for a valid property, or actually reading it */
				if (DataLength != NULL) {
					*DataLength = 3;

					Data[2] = (CurrentAudioSampleFrequency >> 16);
					Data[1] = (CurrentAudioSampleFrequency >> 8);
					Data[0] = (CurrentAudioSampleFrequency &  0xFF);
				}

				return true;
			}
		}
	}

	return false;
}

/** Audio class driver callback for the setting and retrieval of streaming interface properties. This callback must be implemented
 *  in the user application to handle property manipulations on streaming audio interfaces.
 *
 */
bool CALLBACK_Audio_Device_GetSetInterfaceProperty(USB_ClassInfo_Audio_Device_t *const AudioInterfaceInfo,
												   const uint8_t Property,
												   const uint8_t EntityAddress,
												   const uint16_t Parameter,
												   uint16_t *const DataLength,
												   uint8_t *Data)
{
	/* No audio interface entities in the device descriptor, thus no properties to get or set. */
	return false;
}
