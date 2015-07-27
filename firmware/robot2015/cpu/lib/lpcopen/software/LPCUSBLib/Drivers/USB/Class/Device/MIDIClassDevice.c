/*
 * @brief Device mode driver for the library USB MIDI Class driver
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


#define  __INCLUDE_FROM_USB_DRIVER
#include "../../Core/USBMode.h"

#if defined(USB_CAN_BE_DEVICE)

#define  __INCLUDE_FROM_MIDI_DRIVER
#define  __INCLUDE_FROM_MIDI_DEVICE_C
#include "MIDIClassDevice.h"

bool MIDI_Device_ConfigureEndpoints(USB_ClassInfo_MIDI_Device_t* const MIDIInterfaceInfo)
{
	// memset(&MIDIInterfaceInfo->State, 0x00, sizeof(MIDIInterfaceInfo->State));

	for (uint8_t EndpointNum = 1; EndpointNum < ENDPOINT_TOTAL_ENDPOINTS(MIDIInterfaceInfo->Config.PortNumber); EndpointNum++)
	{
		uint16_t Size;
		uint8_t  Type;
		uint8_t  Direction;
		bool     DoubleBanked;

		if (EndpointNum == MIDIInterfaceInfo->Config.DataINEndpointNumber)
		{
			Size         = MIDIInterfaceInfo->Config.DataINEndpointSize;
			Direction    = ENDPOINT_DIR_IN;
			Type         = EP_TYPE_BULK;
			DoubleBanked = MIDIInterfaceInfo->Config.DataINEndpointDoubleBank;
		}
		else if (EndpointNum == MIDIInterfaceInfo->Config.DataOUTEndpointNumber)
		{
			Size         = MIDIInterfaceInfo->Config.DataOUTEndpointSize;
			Direction    = ENDPOINT_DIR_OUT;
			Type         = EP_TYPE_BULK;
			DoubleBanked = MIDIInterfaceInfo->Config.DataOUTEndpointDoubleBank;
		}
		else
		{
			continue;
		}

		if (!(Endpoint_ConfigureEndpoint(MIDIInterfaceInfo->Config.PortNumber, EndpointNum, Type, Direction, Size,
		                                 DoubleBanked ? ENDPOINT_BANK_DOUBLE : ENDPOINT_BANK_SINGLE)))
		{
			return false;
		}
	}

	return true;
}

void MIDI_Device_USBTask(USB_ClassInfo_MIDI_Device_t* const MIDIInterfaceInfo)
{
	if (USB_DeviceState[MIDIInterfaceInfo->Config.PortNumber] != DEVICE_STATE_Configured)
	  return;

	#if !defined(NO_CLASS_DRIVER_AUTOFLUSH)
	MIDI_Device_Flush(MIDIInterfaceInfo);
	#endif
}

uint8_t MIDI_Device_SendEventPacket(USB_ClassInfo_MIDI_Device_t* const MIDIInterfaceInfo,
                                    const MIDI_EventPacket_t* const Event)
{
	if (USB_DeviceState[MIDIInterfaceInfo->Config.PortNumber] != DEVICE_STATE_Configured)
	  return ENDPOINT_RWSTREAM_DeviceDisconnected;

	uint8_t ErrorCode;

	Endpoint_SelectEndpoint(MIDIInterfaceInfo->Config.PortNumber, MIDIInterfaceInfo->Config.DataINEndpointNumber);

	if ((ErrorCode = Endpoint_Write_Stream_LE(MIDIInterfaceInfo->Config.PortNumber, Event, sizeof(MIDI_EventPacket_t), NULL)) != ENDPOINT_RWSTREAM_NoError)
	  return ErrorCode;

	if (!(Endpoint_IsReadWriteAllowed(MIDIInterfaceInfo->Config.PortNumber)))
	  Endpoint_ClearIN(MIDIInterfaceInfo->Config.PortNumber);

	return ENDPOINT_RWSTREAM_NoError;
}

uint8_t MIDI_Device_Flush(USB_ClassInfo_MIDI_Device_t* const MIDIInterfaceInfo)
{
	if (USB_DeviceState[MIDIInterfaceInfo->Config.PortNumber] != DEVICE_STATE_Configured)
	  return ENDPOINT_RWSTREAM_DeviceDisconnected;

	uint8_t ErrorCode;

	Endpoint_SelectEndpoint(MIDIInterfaceInfo->Config.PortNumber, MIDIInterfaceInfo->Config.DataINEndpointNumber);

	if (Endpoint_BytesInEndpoint(MIDIInterfaceInfo->Config.PortNumber))
	{
		Endpoint_ClearIN(MIDIInterfaceInfo->Config.PortNumber);

		if ((ErrorCode = Endpoint_WaitUntilReady()) != ENDPOINT_READYWAIT_NoError)
		  return ErrorCode;
	}

	return ENDPOINT_READYWAIT_NoError;
}

bool MIDI_Device_ReceiveEventPacket(USB_ClassInfo_MIDI_Device_t* const MIDIInterfaceInfo,
                                    MIDI_EventPacket_t* const Event)
{
	if (USB_DeviceState[MIDIInterfaceInfo->Config.PortNumber] != DEVICE_STATE_Configured)
	  return false;

	Endpoint_SelectEndpoint(MIDIInterfaceInfo->Config.PortNumber, MIDIInterfaceInfo->Config.DataOUTEndpointNumber);

	if (!(Endpoint_IsReadWriteAllowed(MIDIInterfaceInfo->Config.PortNumber)))
	  return false;

	Endpoint_Read_Stream_LE(MIDIInterfaceInfo->Config.PortNumber, Event, sizeof(MIDI_EventPacket_t), NULL);

	if (!(Endpoint_IsReadWriteAllowed(MIDIInterfaceInfo->Config.PortNumber)))
	  Endpoint_ClearOUT(MIDIInterfaceInfo->Config.PortNumber);

	return true;
}

#endif

