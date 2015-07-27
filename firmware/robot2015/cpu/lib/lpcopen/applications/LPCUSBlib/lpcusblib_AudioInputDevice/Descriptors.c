/*
 * @brief USB Device Descriptors, for library use when in USB device mode. Descriptors are special
 *        computer-readable structures which the host requests upon device enumeration, to determine
 *        the device's capabilities and functions
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

#include "Descriptors.h"

/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/

/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/

/** Device descriptor structure. This descriptor, located in FLASH memory, describes the overall
 *  device characteristics, including the supported USB version, control endpoint size and the
 *  number of device configurations. The descriptor is read out by the USB host when the enumeration
 *  process begins.
 */
USB_Descriptor_Device_t DeviceDescriptor = {
	.Header                 = {.Size = sizeof(USB_Descriptor_Device_t), .Type = DTYPE_Device},

	.USBSpecification       = VERSION_BCD(02.00),
	.Class                  = USB_CSCP_NoDeviceClass,
	.SubClass               = USB_CSCP_NoDeviceSubclass,
	.Protocol               = USB_CSCP_NoDeviceProtocol,

	.Endpoint0Size          = FIXED_CONTROL_ENDPOINT_SIZE,

	.VendorID               = 0x1fc9,
	.ProductID              = 0x2047,
	.ReleaseNumber          = VERSION_BCD(00.02),

	.ManufacturerStrIndex   = 0x01,
	.ProductStrIndex        = 0x02,
	.SerialNumStrIndex      = NO_DESCRIPTOR,

	.NumberOfConfigurations = FIXED_NUM_CONFIGURATIONS
};

/** Configuration descriptor structure. This descriptor, located in FLASH memory, describes the usage
 *  of the device in one of its supported configurations, including information about any device interfaces
 *  and endpoints. The descriptor is read out by the USB host during the enumeration process when selecting
 *  a configuration so that the host may correctly communicate with the USB device.
 */
USB_Descriptor_Configuration_t ConfigurationDescriptor = {
	.Config = {
		.Header                   = {.Size = sizeof(USB_Descriptor_Configuration_Header_t), .Type = DTYPE_Configuration},

		.TotalConfigurationSize   = sizeof(USB_Descriptor_Configuration_t) - 1,		// termination byte not included in size
		.TotalInterfaces          = 2,

		.ConfigurationNumber      = 1,
		.ConfigurationStrIndex    = NO_DESCRIPTOR,

		.ConfigAttributes         = (USB_CONFIG_ATTR_BUSPOWERED | USB_CONFIG_ATTR_SELFPOWERED),

		.MaxPowerConsumption      = USB_CONFIG_POWER_MA(100)
	},

	.Audio_ControlInterface = {
		.Header                   = {.Size = sizeof(USB_Descriptor_Interface_t), .Type = DTYPE_Interface},

		.InterfaceNumber          = 0,
		.AlternateSetting         = 0,

		.TotalEndpoints           = 0,

		.Class                    = AUDIO_CSCP_AudioClass,
		.SubClass                 = AUDIO_CSCP_ControlSubclass,
		.Protocol                 = AUDIO_CSCP_ControlProtocol,

		.InterfaceStrIndex        = NO_DESCRIPTOR
	},

	.Audio_ControlInterface_SPC = {
		.Header                   = {.Size = sizeof(USB_Audio_Descriptor_Interface_AC_t), .Type = DTYPE_CSInterface},
		.Subtype                  = AUDIO_DSUBTYPE_CSInterface_Header,

		.ACSpecification          = VERSION_BCD(01.00),
		.TotalLength              = (sizeof(USB_Audio_Descriptor_Interface_AC_t) +
									 sizeof(USB_Audio_Descriptor_InputTerminal_t) +
									 sizeof(USB_Audio_Descriptor_OutputTerminal_t)),

		.InCollection             = 1,
		.InterfaceNumber          = 1,
	},

	.Audio_InputTerminal = {
		.Header                   = {.Size = sizeof(USB_Audio_Descriptor_InputTerminal_t), .Type = DTYPE_CSInterface},
		.Subtype                  = AUDIO_DSUBTYPE_CSInterface_InputTerminal,

		.TerminalID               = 0x01,
		.TerminalType             = AUDIO_TERMINAL_IN_MIC,
		.AssociatedOutputTerminal = 0x00,

		.TotalChannels            = 1,
		.ChannelConfig            = 0,

		.ChannelStrIndex          = NO_DESCRIPTOR,
		.TerminalStrIndex         = NO_DESCRIPTOR
	},

	.Audio_OutputTerminal = {
		.Header                   = {.Size = sizeof(USB_Audio_Descriptor_OutputTerminal_t), .Type = DTYPE_CSInterface},
		.Subtype                  = AUDIO_DSUBTYPE_CSInterface_OutputTerminal,

		.TerminalID               = 0x02,
		.TerminalType             = AUDIO_TERMINAL_STREAMING,
		.AssociatedInputTerminal  = 0x00,

		.SourceID                 = 0x01,

		.TerminalStrIndex         = NO_DESCRIPTOR
	},

	.Audio_StreamInterface_Alt0 = {
		.Header                   = {.Size = sizeof(USB_Descriptor_Interface_t), .Type = DTYPE_Interface},

		.InterfaceNumber          = 1,
		.AlternateSetting         = 0,

		.TotalEndpoints           = 0,

		.Class                    = AUDIO_CSCP_AudioClass,
		.SubClass                 = AUDIO_CSCP_AudioStreamingSubclass,
		.Protocol                 = AUDIO_CSCP_StreamingProtocol,

		.InterfaceStrIndex        = NO_DESCRIPTOR
	},

	.Audio_StreamInterface_Alt1 = {
		.Header                   = {.Size = sizeof(USB_Descriptor_Interface_t), .Type = DTYPE_Interface},

		.InterfaceNumber          = 1,
		.AlternateSetting         = 1,

		.TotalEndpoints           = 1,

		.Class                    = AUDIO_CSCP_AudioClass,
		.SubClass                 = AUDIO_CSCP_AudioStreamingSubclass,
		.Protocol                 = AUDIO_CSCP_StreamingProtocol,

		.InterfaceStrIndex        = NO_DESCRIPTOR
	},

	.Audio_StreamInterface_SPC = {
		.Header                   = {.Size = sizeof(USB_Audio_Descriptor_Interface_AS_t), .Type = DTYPE_CSInterface},
		.Subtype                  = AUDIO_DSUBTYPE_CSInterface_General,

		.TerminalLink             = 0x02,

		.FrameDelay               = 1,
		.AudioFormat              = 0x0001
	},

	.Audio_AudioFormat = {
		.Header                   = {.Size = sizeof(USB_Audio_Descriptor_Format_t) +
											 sizeof(ConfigurationDescriptor.Audio_AudioFormatSampleRates),
									 .Type = DTYPE_CSInterface},
		.Subtype                  = AUDIO_DSUBTYPE_CSInterface_FormatType,

		.FormatType               = 0x01,
		.Channels                 = 0x01,

		.SubFrameSize             = 0x02,
		.BitResolution            = 16,

		.TotalDiscreteSampleRates =
			(sizeof(ConfigurationDescriptor.Audio_AudioFormatSampleRates) / sizeof(USB_Audio_SampleFreq_t))
	},

	.Audio_AudioFormatSampleRates = {
		AUDIO_SAMPLE_FREQ(8000),
		AUDIO_SAMPLE_FREQ(11025),
		AUDIO_SAMPLE_FREQ(22050),
		AUDIO_SAMPLE_FREQ(44100),
		AUDIO_SAMPLE_FREQ(48000),
	},

	.Audio_StreamEndpoint = {
		.Endpoint = {
			.Header              = {.Size = sizeof(USB_Audio_Descriptor_StreamEndpoint_Std_t), .Type = DTYPE_Endpoint},

			.EndpointAddress     = (ENDPOINT_DIR_IN | AUDIO_STREAM_EPNUM),
			.Attributes          = (EP_TYPE_ISOCHRONOUS | ENDPOINT_ATTR_SYNC | ENDPOINT_USAGE_DATA),
			.EndpointSize        = AUDIO_STREAM_EPSIZE,
			.PollingIntervalMS   = 0x01
		},

		.Refresh                  = 0,
		.SyncEndpointNumber       = 0
	},

	.Audio_StreamEndpoint_SPC = {
		.Header                   =
		{.Size = sizeof(USB_Audio_Descriptor_StreamEndpoint_Spc_t), .Type = DTYPE_CSEndpoint},
		.Subtype                  = AUDIO_DSUBTYPE_CSEndpoint_General,

		.Attributes               = (AUDIO_EP_ACCEPTS_SMALL_PACKETS | AUDIO_EP_SAMPLE_FREQ_CONTROL),

		.LockDelayUnits           = 0x00,
		.LockDelay                = 0x0000
	},
	.Audio_Termination = 0x00
};

/** Language descriptor structure. This descriptor, located in FLASH memory, is returned when the host requests
 *  the string descriptor with index 0 (the first index). It is actually an array of 16-bit integers, which indicate
 *  via the language ID table available at USB.org what languages the device supports for its string descriptors.
 */
uint8_t LanguageString[] = {
	USB_STRING_LEN(1),
	DTYPE_String,
	WBVAL(LANGUAGE_ID_ENG),
};
USB_Descriptor_String_t *LanguageStringPtr = (USB_Descriptor_String_t *) LanguageString;

/** Manufacturer descriptor string. This is a Unicode string containing the manufacturer's details in human readable
 *  form, and is read out upon request by the host when the appropriate string ID is requested, listed in the Device
 *  Descriptor.
 */
uint8_t ManufacturerString[] = {
	USB_STRING_LEN(3),
	DTYPE_String,
	WBVAL('N'),
	WBVAL('X'),
	WBVAL('P'),
};
USB_Descriptor_String_t *ManufacturerStringPtr = (USB_Descriptor_String_t *) ManufacturerString;

/** Product descriptor string. This is a Unicode string containing the product's details in human readable form,
 *  and is read out upon request by the host when the appropriate string ID is requested, listed in the Device
 *  Descriptor.
 */
uint8_t ProductString[] = {
	USB_STRING_LEN(26),
	DTYPE_String,
	WBVAL('L'),
	WBVAL('P'),
	WBVAL('C'),
	WBVAL('U'),
	WBVAL('S'),
	WBVAL('B'),
	WBVAL('l'),
	WBVAL('i'),
	WBVAL('b'),
	WBVAL(' '),
	WBVAL('A'),
	WBVAL('u'),
	WBVAL('d'),
	WBVAL('i'),
	WBVAL('o'),
	WBVAL(' '),
	WBVAL('I'),
	WBVAL('n'),
	WBVAL('p'),
	WBVAL('u'),
	WBVAL('t'),
	WBVAL(' '),
	WBVAL('D'),
	WBVAL('e'),
	WBVAL('m'),
	WBVAL('o'),
};
USB_Descriptor_String_t *ProductStringPtr = (USB_Descriptor_String_t *) ProductString;

/*****************************************************************************
 * Private functions
 ****************************************************************************/

/*****************************************************************************
 * Public functions
 ****************************************************************************/

/** This function is called by the library when in device mode, and must be overridden (see library "USB Descriptors"
 *  documentation) by the application code so that the address and size of a requested descriptor can be given
 *  to the USB library. When the device receives a Get Descriptor request on the control endpoint, this function
 *  is called so that the descriptor details can be passed back and the appropriate descriptor sent back to the
 *  USB host.
 */
uint16_t CALLBACK_USB_GetDescriptor(uint8_t corenum,
									const uint16_t wValue,
									const uint8_t wIndex,
									const void * *const DescriptorAddress)
{
	const uint8_t  DescriptorType   = (wValue >> 8);
	const uint8_t  DescriptorNumber = (wValue & 0xFF);

	const void *Address = NULL;
	uint16_t    Size    = NO_DESCRIPTOR;

	switch (DescriptorType) {
	case DTYPE_Device:
		Address = &DeviceDescriptor;
		Size    = sizeof(USB_Descriptor_Device_t);
		break;

	case DTYPE_Configuration:
		Address = &ConfigurationDescriptor;
		Size    = sizeof(USB_Descriptor_Configuration_t);
		break;

	case DTYPE_String:
		switch (DescriptorNumber) {
		case 0x00:
			Address = LanguageStringPtr;
			Size    = pgm_read_byte(&LanguageStringPtr->Header.Size);
			break;

		case 0x01:
			Address = ManufacturerStringPtr;
			Size    = pgm_read_byte(&ManufacturerStringPtr->Header.Size);
			break;

		case 0x02:
			Address = ProductStringPtr;
			Size    = pgm_read_byte(&ProductStringPtr->Header.Size);
			break;
		}

		break;
	}

	*DescriptorAddress = Address;
	return Size;
}
