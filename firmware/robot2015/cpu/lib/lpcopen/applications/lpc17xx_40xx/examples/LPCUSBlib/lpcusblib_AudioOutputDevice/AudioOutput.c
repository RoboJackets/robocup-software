/*
 * @brief Make your board becomes a USB speaker
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

#include "AudioOutput.h"
#if defined(USB_DEVICE_ROM_DRIVER)
//#include "usbd_adcuser.h"
#endif
/** LPCUSBlib Audio Class driver interface configuration and state information. This structure is
 *  passed to all Audio Class driver functions, so that multiple instances of the same class
 *  within a device can be differentiated from one another.
 */
USB_ClassInfo_Audio_Device_t Speaker_Audio_Interface = {
	.Config = {
		.StreamingInterfaceNumber = 1,

		.DataOUTEndpointNumber    = AUDIO_STREAM_EPNUM,
		.DataOUTEndpointSize      = AUDIO_STREAM_EPSIZE,
		.PortNumber = 0,
	},
};

/** Max Sample Frequency. */
#define AUDIO_MAX_SAMPLE_FREQ   48000
/** if audio buffer count is over this value, i2s will run in higher speed */
#define AUDIO_SPEEP_UP_TRIGGER			(audio_buffer_size*5/8)
/** if audio buffer count is under this value, i2s will run in normal speed */
#define AUDIO_NORMAL_SPEED_TRIGGER	(audio_buffer_size*3/8)
/** Current audio sampling frequency of the streaming audio endpoint. */
uint32_t CurrentAudioSampleFrequency = AUDIO_MAX_SAMPLE_FREQ;

/**
 * Audio API
 */
/** Audio max packet count. */
#define AUDIO_MAX_PC    10
PRAGMA_ALIGN_4
uint8_t audio_buffer[2048*2] ATTR_ALIGNED(4) __BSS(USBRAM_SECTION);
uint32_t audio_buffer_size = 0;
uint32_t audio_buffer_rd_index = 0;
uint32_t audio_buffer_wr_index = 0;
uint32_t audio_buffer_count = 0;

#if defined(__LPC177X_8X__) || defined(__LPC407X_8X__)
typedef struct {
	uint8_t BITRATE;
	struct {
		uint8_t X,Y;
	} RATEUP;
	struct {
		uint8_t X,Y;
	} RATEDOWN;
} I2S_RATE_CONFIG;

static uint32_t I2S_SpeedConfig_index;

I2S_RATE_CONFIG I2S_SpeedConfig[] = 
{
	/* BITRATE	RATE_Up(x,y)	RATE_Down(x,y) */
	/* 8000 */
	{0x3F,	{0x2D,	0xF7},	{0x02,	0x0B}},
	/* 11025 */
	{0x3F,	{0x40,	0xFF},	{0x01,	0x04}},
	/* 16000 unsupport */
	{0,	{0,	0},	{0,	0}},
	/* 22050 */
	{0x3F,	{0x80,	0xFF},	{0x01,	0x02}},
	/* 32000 unsupport */
	{0,	{0,	0},	{0,	0}},
	/* 44100 */
	{0x3E,	{0xF4,	0xF7},	{0x51,	0x52}},
	/* 48000 */
	{0x39,	{0x62,	0x63},	{0xC3,	0xC5}}
	
};

Status I2S_RateFind(LPC_I2S_T *I2Sx, Chip_I2S_Audio_Format_T *audio_format, I2S_RATE_CONFIG *I2S_Config)
{
	uint32_t pClk;
	uint32_t x, y;
	uint64_t divider;
	uint16_t dif;
	uint16_t x_divide = 0, y_divide_down = 0, y_divide_up = 0;
	uint32_t N;
	uint32_t err, ErrorOptimal_down = 0xFFFF, ErrorOptimal_up = 0xFFFF;

	pClk = (uint64_t)(Chip_Clock_GetPeripheralClockRate()*2);
	
	/* find N that make x/y <= 1 -> divider <= 2^16 */
	for (N = 64; N > 0; N--) {
		/* divider is a fixed point number with 16 fractional bits */
		divider = (((uint64_t)(audio_format->SampleRate) * 2 * (audio_format->WordWidth) * 2) << 16) * N / pClk;
		if (divider < (1 << 16)) {
			break;
		}
	}
	if (N == 0) {
		return ERROR;
	}
	//divider *= N;
	for (y = 255; y > 0; y--) {
		x = y * divider;
		if (x & (0xFF000000)) {
			continue;
		}
		dif = x & 0xFFFF;
		if (dif > 0x8000) {
			/* mark this case to plus 1 to x value */
			err = (0x10000 - dif);
			/* in this case x = 255 + 1 */
			if((x & 0x00FF0000) == 0x00FF0000) continue;
			if (err < ErrorOptimal_up ) {
				ErrorOptimal_up = err;
				y_divide_up = y;
			}
		}
		else {
			err = dif;
			if (err < ErrorOptimal_down) {
				ErrorOptimal_down = err;
				y_divide_down = y;
			}
		}
	}
	I2S_Config->BITRATE = N - 1;
	x_divide = ((uint64_t)y_divide_up * (audio_format->SampleRate) * 2 * (audio_format->WordWidth) * N * 2) / pClk;
	x_divide += 1;
	if (x_divide >= 256) {
		x_divide = 0xFF;
	}
	if (x_divide == 0) {
		x_divide = 1;
	}
	I2S_Config->RATEUP.X = x_divide;
	I2S_Config->RATEUP.Y = y_divide_up;
	
	x_divide = ((uint64_t)y_divide_down * (audio_format->SampleRate) * 2 * (audio_format->WordWidth) * N * 2) / pClk;
	if (x_divide >= 256) {
		x_divide = 0xFF;
	}
	if (x_divide == 0) {
		x_divide = 1;
	}
	I2S_Config->RATEDOWN.X = x_divide;
	I2S_Config->RATEDOWN.Y = y_divide_down;
	return SUCCESS;
}
#endif

void Audio_Reset_Data_Buffer(void)
{
	audio_buffer_count = 0;
	audio_buffer_wr_index = 0;
	audio_buffer_rd_index = 0;
}

uint32_t Audio_Get_ISO_Buffer_Address(uint32_t last_packet_size)
{
	audio_buffer_wr_index += last_packet_size;
	audio_buffer_count += last_packet_size;
	if (audio_buffer_count > audio_buffer_size) {
		Audio_Reset_Data_Buffer();
	}
	if (audio_buffer_wr_index >= audio_buffer_size) {
		audio_buffer_wr_index -= audio_buffer_size;
	}
	return (uint32_t) &audio_buffer[audio_buffer_wr_index];
}

void Audio_Init(uint32_t samplefreq)
{
#if defined(__LPC177X_8X__) || defined(__LPC407X_8X__)
	Chip_I2S_Audio_Format_T audio_Confg;
	audio_Confg.SampleRate = samplefreq;
	audio_Confg.ChannelNumber = 2;	// 1 is mono, 2 is stereo
	audio_Confg.WordWidth = 16;	// 8, 16 or 32 bits
	Board_Audio_Init(LPC_I2S, UDA1380_LINE_IN);
	Chip_I2S_Init(LPC_I2S);
	Chip_I2S_Config(LPC_I2S, I2S_TX_MODE, &audio_Confg);
	Chip_I2S_Stop(LPC_I2S, I2S_TX_MODE);
	Chip_I2S_DisableMute(LPC_I2S);
	Chip_I2S_Start(LPC_I2S, I2S_TX_MODE);
	Chip_I2S_Int_Cmd(LPC_I2S, I2S_TX_MODE,    ENABLE, 4);
	NVIC_EnableIRQ(I2S_IRQn);

	Audio_Reset_Data_Buffer();
	switch (samplefreq) {
	case 11025:
		I2S_SpeedConfig_index = 1;
		audio_buffer_size = 1764;
		break;
	case 22050:
		I2S_SpeedConfig_index = 3;
		audio_buffer_size = 1764;
		break;
	case 44100:
		I2S_SpeedConfig_index = 5;
		audio_buffer_size = 1764;
		break;

	case 8000:
		I2S_SpeedConfig_index = 0;
	audio_buffer_size = samplefreq * 4 * AUDIO_MAX_PC / 1000;
		break;
	case 16000:
		I2S_SpeedConfig_index = 2;
		audio_buffer_size = samplefreq * 4 * AUDIO_MAX_PC / 1000;
		break;
	case 32000:
		I2S_SpeedConfig_index = 4;
		audio_buffer_size = samplefreq * 4 * AUDIO_MAX_PC / 1000;
		break;
	case 48000:
		I2S_SpeedConfig_index = 6;
	default:
		audio_buffer_size = samplefreq * 4 * AUDIO_MAX_PC / 1000;
		break;
	}
	IP_I2S_SetBitRate(LPC_I2S, I2S_TX_MODE, I2S_SpeedConfig[I2S_SpeedConfig_index].BITRATE);
	IP_I2S_SetXYDivider(LPC_I2S, I2S_TX_MODE, 
														I2S_SpeedConfig[I2S_SpeedConfig_index].RATEDOWN.X, 
														I2S_SpeedConfig[I2S_SpeedConfig_index].RATEDOWN.Y);
	audio_buffer_size*=2;
#else
	volatile uint32_t pclk;
	Chip_DAC_Init(LPC_DAC);
	Chip_Clock_SetPCLKDiv(SYSCTL_PCLK_DAC, SYSCTL_CLKDIV_1);
	Chip_TIMER_Init(LPC_TIMER0);
	Chip_TIMER_Reset(LPC_TIMER0);
	Chip_TIMER_MatchEnableInt(LPC_TIMER0, 0);
	pclk = Chip_Clock_GetPeripheralClockRate(SYSCTL_PCLK_TIMER0);
	Chip_TIMER_SetMatch(LPC_TIMER0, 0, ((pclk / samplefreq) - 1));
	Chip_TIMER_ResetOnMatchEnable(LPC_TIMER0, 0);
	Chip_TIMER_Enable(LPC_TIMER0);
	NVIC_EnableIRQ(TIMER0_IRQn);
	switch(samplefreq)
	{
		case 11025:
		case 22050:
		case 44100:
			audio_buffer_size = 1764;
			break;
		case 8000:
		case 16000:
		case 32000:
		case 48000:
		default:
			audio_buffer_size = samplefreq * 4 * AUDIO_MAX_PC / 1000;
			break;
	}
#endif
}

void Audio_DeInit(void)
{
#if defined(__LPC177X_8X__) || defined(__LPC407X_8X__)
	NVIC_DisableIRQ(I2S_IRQn);
#endif
}

#if defined(__LPC177X_8X__) || defined(__LPC407X_8X__)
void I2S_IRQHandler(void)
{
	uint32_t txlevel, i;
	static bool double_speed = false;
	static uint32_t sample = 0;
	txlevel = Chip_I2S_GetLevel(LPC_I2S, I2S_TX_MODE);
	if (txlevel <= 4) {
		for (i = 0; i < 8 - txlevel; i++) {
			if (audio_buffer_count >= 4) {	/*has enough data */
				audio_buffer_count -= 4;
				sample = *(uint32_t *) (audio_buffer + audio_buffer_rd_index);
				audio_buffer_rd_index += 4;
				if (audio_buffer_rd_index >= audio_buffer_size) {
					audio_buffer_rd_index -= audio_buffer_size;
				}
			}
			Chip_I2S_Send(LPC_I2S, sample);
		}
			/*Skip some samples if buffer run writting too fast. */
		if(audio_buffer_size != 0)
 		{
 			if(audio_buffer_count >= AUDIO_SPEEP_UP_TRIGGER){
 				if(!double_speed) {
					IP_I2S_SetXYDivider(LPC_I2S, I2S_TX_MODE, 
														I2S_SpeedConfig[I2S_SpeedConfig_index].RATEUP.X, 
														I2S_SpeedConfig[I2S_SpeedConfig_index].RATEUP.Y);
					double_speed = true;
				}
 			}else if(audio_buffer_count < AUDIO_NORMAL_SPEED_TRIGGER){
 				if(double_speed){
					IP_I2S_SetXYDivider(LPC_I2S, I2S_TX_MODE, 
														I2S_SpeedConfig[I2S_SpeedConfig_index].RATEDOWN.X, 
														I2S_SpeedConfig[I2S_SpeedConfig_index].RATEDOWN.Y);
					double_speed = false;
				}
 			}
 		}
	}
}
#else
void TIMER0_IRQHandler(void)
{
		uint32_t val;
		int32_t sample;
		if(audio_buffer_count >= 2) /*has enough data */
		{
			audio_buffer_count -= 2;
			sample = *(int16_t *)(audio_buffer + audio_buffer_rd_index);
			audio_buffer_rd_index+=2;
			if(audio_buffer_rd_index >= audio_buffer_size)
				audio_buffer_rd_index -= audio_buffer_size;
		}else{
			sample = 0;
		}
		val = sample>>6;
		val &= 0x3FF;
		val += (0x01 << 9);
		Chip_DAC_UpdateValue(LPC_DAC, val);
		if((audio_buffer_size != 0) && (audio_buffer_count >= (audio_buffer_size/2)))
		{
			audio_buffer_count-=2;
			audio_buffer_rd_index+=2;
			if(audio_buffer_rd_index >= audio_buffer_size)
				audio_buffer_rd_index -= audio_buffer_size;
		}
		Chip_TIMER_ClearMatch(LPC_TIMER0, 0);
}
#endif

/** This callback function provides iso buffer address for HAL iso transfer processing.
 */
uint32_t CALLBACK_HAL_GetISOBufferAddress(const uint32_t EPNum, uint32_t *last_packet_size) {

	/* Check if this is audio stream endpoint */
	if (EPNum == AUDIO_STREAM_EPNUM) {
		return Audio_Get_ISO_Buffer_Address(*last_packet_size);
	}
	else {return 0; }
}

/** Main program entry point. This routine contains the overall program flow, including initial
 *  setup of all components and the main program loop.
 */
int main(void)
{
#if defined(__LPC177X_8X__) || defined(__LPC407X_8X__)
	Chip_I2S_Audio_Format_T audio_Confg;
#endif
	SetupHardware();
#if defined(__LPC177X_8X__) || defined(__LPC407X_8X__)
	audio_Confg.ChannelNumber = 2;	//stereo
	audio_Confg.WordWidth = 16;	//16 bits	
	/* Build I2S config table */
	audio_Confg.SampleRate = 8000;
	I2S_RateFind(LPC_I2S, &audio_Confg, &I2S_SpeedConfig[0]);
	audio_Confg.SampleRate = 11025;
	I2S_RateFind(LPC_I2S, &audio_Confg, &I2S_SpeedConfig[1]);
	audio_Confg.SampleRate = 22050;
	I2S_RateFind(LPC_I2S, &audio_Confg, &I2S_SpeedConfig[3]);
	/* Although the founded rate is very accuracy, 
	the XTAL doesn't always match its frequency.
	Increase the sample rate a little bit will really meet the USB speed. */
#if defined(__LPC43XX__ )
	audio_Confg.SampleRate = 44101; /* a bit higher for 204Mhz device */
#else
	audio_Confg.SampleRate = 44100;
#endif
	I2S_RateFind(LPC_I2S, &audio_Confg, &I2S_SpeedConfig[5]);
#if defined(__LPC43XX__ )
	audio_Confg.SampleRate = 48001; /* a bit higher for 204Mhz device */
#else
	audio_Confg.SampleRate = 48000;
#endif
	I2S_RateFind(LPC_I2S, &audio_Confg, &I2S_SpeedConfig[6]);
#endif
	
#if defined(USB_DEVICE_ROM_DRIVER)
	UsbdAdc_Init(&Speaker_Audio_Interface);
#endif

	for (;; ) {
#if !defined(USB_DEVICE_ROM_DRIVER)
		Audio_Device_USBTask(&Speaker_Audio_Interface);
		USB_USBTask(Speaker_Audio_Interface.Config.PortNumber,USB_MODE_Device);
#endif
	}
}

/** Configures the board hardware and chip peripherals for the demo's functionality. */
void SetupHardware(void)
{
	Board_Init();
	USB_Init(Speaker_Audio_Interface.Config.PortNumber, USB_MODE_Device);
}

#if !defined(USB_DEVICE_ROM_DRIVER)
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

	ConfigSuccess &= Audio_Device_ConfigureEndpoints(&Speaker_Audio_Interface);

	//	LEDs_SetAllLEDs(ConfigSuccess ? LEDMASK_USB_READY : LEDMASK_USB_ERROR);
}

/** Event handler for the library USB Control Request reception event. */
void EVENT_USB_Device_ControlRequest(void)
{
	Audio_Device_ProcessControlRequest(&Speaker_Audio_Interface);
}

void EVENT_Audio_Device_StreamStartStop(USB_ClassInfo_Audio_Device_t *const AudioInterfaceInfo)
{
	/* reset audio buffer */
	Audio_Reset_Data_Buffer();
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
	if (EndpointAddress == (ENDPOINT_DIR_OUT | Speaker_Audio_Interface.Config.DataOUTEndpointNumber)) {
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
					Audio_DeInit();
					Audio_Init(CurrentAudioSampleFrequency);
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

#else

#endif
