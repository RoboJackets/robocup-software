/*
 * @brief Audio Output Host Example
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

#include "AudioOutputHost.h"

/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/

/** LPCUSBlib Audio Class driver interface configuration and state information. This structure is
 *  passed to all Audio Class driver functions, so that multiple instances of the same class
 *  within a device can be differentiated from one another.
 */
static USB_ClassInfo_Audio_Host_t Speaker_Audio_Interface = {
	.Config = {
		.DataOUTPipeNumber = 1,
		.PortNumber = 0,
	},
};

/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/

/*****************************************************************************
 * Private functions
 ****************************************************************************/

/* Timer setup for audio waveform */
static void Init_Timer(uint32_t freq)
{
	/* Enable timer 1 clock and reset */
	Chip_TIMER_Init(LPC_TIMER1);

	/* Timer setup for match and interrupt at TICKRATE */
	Chip_TIMER_Reset(LPC_TIMER1);
	Chip_TIMER_MatchEnableInt(LPC_TIMER1, 1);
	Chip_TIMER_SetMatch(LPC_TIMER1, 1, (1000000 / freq));
	Chip_TIMER_ResetOnMatchEnable(LPC_TIMER1, 1);
	Chip_TIMER_Enable(LPC_TIMER1);

	NVIC_SetPriority(TIMER1_IRQn, ((0x01 << 3) | 0x01));
	/* Enable timer interrupt */
	NVIC_EnableIRQ(TIMER1_IRQn);
}

static void DeInitTimer(void)
{
	Chip_TIMER_Disable(LPC_TIMER1);
	NVIC_DisableIRQ(TIMER1_IRQn);
}

/* Configures the board hardware and chip peripherals for the demo's functionality. */
static void SetupHardware(void)
{
	Board_Init();
	USB_Init(Speaker_Audio_Interface.Config.PortNumber, USB_MODE_Host);

	/* Hardware Initialization */
	Board_Debug_Init();
	Board_Buttons_Init();

	/* Create a stdio stream for the serial port for stdin and stdout */
	Serial_CreateStream(NULL);
}

/*****************************************************************************
 * Public functions
 ****************************************************************************/

/**
 * @brief	ISR to handle the reloading of the PWM timer with the next sample
 * @return	Nothing
 */
#define USE_TEST_TONE
#if defined(__LPC175X_6X__) || defined(__LPC177X_8X__) || defined(__LPC407X_8X__) || defined(__LPC18XX__) || defined(__LPC43XX__)
void TIMER1_IRQHandler(void)
{
	uint8_t PrevPipe = Pipe_GetCurrentPipe(Speaker_Audio_Interface.Config.PortNumber);
	/* Check that the USB bus is ready for the next sample to write */
	if (Audio_Host_IsReadyForNextSample(&Speaker_Audio_Interface)) {
		int16_t AudioSample;
#if defined(USE_TEST_TONE)
		static uint8_t SquareWaveSampleCount;
		static int16_t CurrentWaveValue;

		/* In test tone mode, generate a square wave at 1/256 of the sample rate */
		if (SquareWaveSampleCount++ == 0xFF) {
			CurrentWaveValue ^= 0x8000;
		}

		/* Only generate audio if the board button is being pressed */
		AudioSample = (Buttons_GetStatus() & BUTTONS_BUTTON1) ? CurrentWaveValue : 0;
#else
		/* Audio sample is ADC value scaled to fit the entire range */
		AudioSample = ((SAMPLE_MAX_RANGE / ADC_MAX_RANGE) * ADC_GetResult());

#if defined(MICROPHONE_BIASED_TO_HALF_RAIL)
		/* Microphone is biased to half rail voltage, subtract the bias from the sample value */
		AudioSample -= (SAMPLE_MAX_RANGE / 2);
#endif /* defined(MICROPHONE_BIASED_TO_HALF_RAIL) */
#endif /* defined(USE_TEST_TONE) */

		Audio_Host_WriteSample16(&Speaker_Audio_Interface, AudioSample);
		Audio_Host_WriteSample16(&Speaker_Audio_Interface, AudioSample);
	}

	Pipe_SelectPipe(Speaker_Audio_Interface.Config.PortNumber, PrevPipe);
}

#endif /* defined(__LPC17XX__) ... */

/**
 * @brief	Main program entry point
 * @return	Nothing
 * @note	This routine configures the hardware required by the application,
 * then enters a loop to run the application tasks in sequence.
 */
int main(void)
{
	SetupHardware();

	DEBUGOUT("Audio Output Host Demo running.\r\n");

	for (;; ) {
		Audio_Host_USBTask(&Speaker_Audio_Interface);
		USB_USBTask(Speaker_Audio_Interface.Config.PortNumber, USB_MODE_Host);
	}
}

/* Event handler for the USB_DeviceAttached event. This indicates that a device
   has been attached to the host, and starts the library USB task to begin the
   enumeration and USB management process. */
void EVENT_USB_Host_DeviceAttached(const uint8_t corenum)
{
	DEBUGOUT(("Device Attached on port %d\r\n"), corenum);
}

/* Event handler for the USB_DeviceUnattached event. This indicates that a device
   has been removed from the host, and stops the library USB task management
   process. */
void EVENT_USB_Host_DeviceUnattached(const uint8_t corenum)
{
	DEBUGOUT(("\r\nDevice Unattached on port %d\r\n"), corenum);
	DeInitTimer();
}

/* Event handler for the USB_DeviceEnumerationComplete event. This indicates that a
   device has been successfully enumerated by the host and is now ready to be used
   by the application. */
void EVENT_USB_Host_DeviceEnumerationComplete(const uint8_t corenum)
{
	uint16_t ConfigDescriptorSize;
	uint8_t  ConfigDescriptorData[512];

	if (USB_Host_GetDeviceConfigDescriptor(corenum, 1, &ConfigDescriptorSize, ConfigDescriptorData,
										   sizeof(ConfigDescriptorData)) != HOST_GETCONFIG_Successful) {
		DEBUGOUT("Error Retrieving Configuration Descriptor.\r\n");

		return;
	}

	Speaker_Audio_Interface.Config.PortNumber = corenum;
	if (Audio_Host_ConfigurePipes(&Speaker_Audio_Interface,
								  ConfigDescriptorSize, ConfigDescriptorData) != AUDIO_ENUMERROR_NoError) {
		DEBUGOUT("Attached Device Not a Valid Audio Output Device.\r\n");

		return;
	}

	if (USB_Host_SetDeviceConfiguration(Speaker_Audio_Interface.Config.PortNumber, 1) != HOST_SENDCONTROL_Successful) {
		DEBUGOUT("Error Setting Device Configuration.\r\n");

		return;
	}

	if (Audio_Host_StartStopStreaming(&Speaker_Audio_Interface, true) != HOST_SENDCONTROL_Successful) {
		DEBUGOUT("Error Enabling Audio Stream.\r\n");

		USB_Host_SetDeviceConfiguration(Speaker_Audio_Interface.Config.PortNumber, 0);
		return;
	}

	USB_Audio_SampleFreq_t SampleRate = AUDIO_SAMPLE_FREQ(48000);
	if (Audio_Host_GetSetEndpointProperty(&Speaker_Audio_Interface, Speaker_Audio_Interface.Config.DataOUTPipeNumber,
										  AUDIO_REQ_SetCurrent, AUDIO_EPCONTROL_SamplingFreq,
										  sizeof(SampleRate), &SampleRate) != HOST_SENDCONTROL_Successful) {
		DEBUGOUT("Error Setting Audio Sampling Frequency.\r\n");

		USB_Host_SetDeviceConfiguration(Speaker_Audio_Interface.Config.PortNumber, 0);
		return;
	}
	DEBUGOUT("Audio Device Enumerated.\r\n");
	Init_Timer(48000);
}

/* Event handler for the USB_HostError event. This indicates that a hardware error
   occurred while in host mode. */
void EVENT_USB_Host_HostError(const uint8_t corenum, const uint8_t ErrorCode)
{
	USB_Disable(corenum, USB_MODE_Host);

	DEBUGOUT(("Host Mode Error\r\n"
			  " -- Error port %d\r\n"
			  " -- Error Code %d\r\n" ), corenum, ErrorCode);

	/* Wait forever */
	for (;; ) {}
}

/* Event handler for the USB_DeviceEnumerationFailed event. This indicates that
   a problem occurred while enumerating an attached USB device. */
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
