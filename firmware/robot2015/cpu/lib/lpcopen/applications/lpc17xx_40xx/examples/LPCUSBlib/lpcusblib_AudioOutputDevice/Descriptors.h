/*
 * @brief Audio device class declarations, definitions for using in application
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

#ifndef _DESCRIPTORS_H_
#define _DESCRIPTORS_H_
		#include "USB.h"
#ifdef __cplusplus
extern "C" {
#endif

/** @defgroup Audio_Output_Device_Descriptor Class descriptors
 * @ingroup LPC17xx_40xx_Audio_Output_Device
 * @{
 */

/**
 * @brief Endpoint number of the Audio isochronous streaming data endpoint
 */
		#if defined(__LPC175X_6X__) || defined(__LPC177X_8X__) || defined(__LPC407X_8X__)
			#define AUDIO_STREAM_EPNUM           3
		#else
			#define AUDIO_STREAM_EPNUM           1
		#endif
/** @brief	Endpoint size in bytes of the Audio isochronous streaming data endpoint. The Windows audio stack requires
 *          at least 192 bytes for correct output, thus the smaller 128 byte maximum endpoint size on some of the smaller
 *          USB models will result in unavoidable distorted output.
 */
		#define AUDIO_STREAM_EPSIZE          ENDPOINT_MAX_SIZE(AUDIO_STREAM_EPNUM)

/** @brief	Type define for the device configuration descriptor structure. This must be defined in the
 *          application code, as the configuration descriptor contains several sub-descriptors which
 *          vary between devices, and which describe the device's usage to the host.
 */
typedef struct {
	USB_Descriptor_Configuration_Header_t     Config;

	// Audio Control Interface
	USB_Descriptor_Interface_t                Audio_ControlInterface;
	USB_Audio_Descriptor_Interface_AC_t       Audio_ControlInterface_SPC;
	USB_Audio_Descriptor_InputTerminal_t      Audio_InputTerminal;
	USB_Audio_Descriptor_OutputTerminal_t     Audio_OutputTerminal;

	// Audio Streaming Interface
	USB_Descriptor_Interface_t                Audio_StreamInterface_Alt0;
	USB_Descriptor_Interface_t                Audio_StreamInterface_Alt1;
	USB_Audio_Descriptor_Interface_AS_t       Audio_StreamInterface_SPC;
	USB_Audio_Descriptor_Format_t             Audio_AudioFormat;
	USB_Audio_SampleFreq_t                    Audio_AudioFormatSampleRates[5];
	USB_Audio_Descriptor_StreamEndpoint_Std_t Audio_StreamEndpoint;
	USB_Audio_Descriptor_StreamEndpoint_Spc_t Audio_StreamEndpoint_SPC;
	unsigned char                             Audio_Termination;
} USB_Descriptor_Configuration_t;

uint16_t CALLBACK_USB_GetDescriptor(uint8_t corenum,
									const uint16_t wValue,
									const uint8_t wIndex,
									const void * *const DescriptorAddress)
ATTR_WARN_UNUSED_RESULT ATTR_NON_NULL_PTR_ARG(4);

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif
