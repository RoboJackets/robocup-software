/*
 * @brief Definitions and declarations of Audio Ouput device example
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

#ifndef _AUDIO_OUTPUT_H_
#define _AUDIO_OUTPUT_H_
		#include "board.h"
		#include "USB.h"
		#include <stdlib.h>
		#include "Descriptors.h"
#ifdef __cplusplus
extern "C" {
#endif

/** @defgroup LPC18xx_43xx_Audio_Output_Device Audio Output Device
 * @ingroup EXAMPLES_USB_18XX43XX
 * <b>Example description</b><br>
 * This example implements an audio interface class device mode device that
 * enumerates as audio device (USB speakers) and sends the samples sent
 * to it from the host to the audio circuitry on the board.
 *
 * On the PC select Control Panel->Hardware and Sound->Sound
 * When the example is first run a new entry in the Sound dialog box
 * will appear titled Speakers and have a description that reads
 * "4-LPCUSBlib Audio Out Demo". Below this description should be
 * the word Ready.
 * 
 * To test the audio output click on this entry in the Sound dialog
 * box. Then click on the Configure button to bring up the Speaker
 * Setup dialog box. In this box press the Test button. Sound should
 * be played through the left side of your headphones first, then
 * the right side.<br>
 * 
 * <b>Special connection requirements</b><br>
 * There are no special connection requirements for this example.<br>
 *
 * <b>Build procedures:</b><br>
 * @ref LPCOPEN_18XX43XX_BUILDPROCS_KEIL<br>
 * @ref LPCOPEN_18XX43XX_BUILDPROCS_IAR<br>
 * @ref LPCOPEN_18XX43XX_BUILDPROCS_XPRESSO<br>
 *
 * <b>Supported boards and board setup:</b><br>
 * @ref LPCOPEN_18XX_BOARD_HITEX1850<br>
 * @ref LPCOPEN_43XX_BOARD_HITEX4350<br>
 * @ref LPCOPEN_18XX_BOARD_KEIL1857<br>
 * @ref LPCOPEN_43XX_BOARD_KEIL4357<br>
 * @ref LPCOPEN_18XX_BOARD_NGX1830<br>
 * @ref LPCOPEN_43XX_BOARD_NGX4330<br>
 *
 * <b>Submitting LPCOpen issues:</b><br>
 * @ref LPCOPEN_COMMUNITY
 * @{
 */

/** @defgroup Audio_Output_Device_Definition Main definitions
 * @{
 */

/**
 * @brief LED definitions
 */
/** LED mask for the library LED driver, to indicate that the USB interface is not ready. */
		#define LEDMASK_USB_NOTREADY      LEDS_LED1

/** LED mask for the library LED driver, to indicate that the USB interface is enumerating. */
		#define LEDMASK_USB_ENUMERATING  (LEDS_LED2 | LEDS_LED3)

/** LED mask for the library LED driver, to indicate that the USB interface is ready. */
		#define LEDMASK_USB_READY        (LEDS_LED2 | LEDS_LED4)

/** LED mask for the library LED driver, to indicate that an error has occurred in the USB interface. */
		#define LEDMASK_USB_ERROR        (LEDS_LED1 | LEDS_LED3)

/**
 * @}
 */

/** @defgroup Audio_Output_Device_Functions Functions prototypes
 * @{
 */

 /**
 * @brief	Hardware setup event callback function
 * @return	None
 * @note	This is the USB HW set up event call back function
 */
void SetupHardware(void);

void EVENT_USB_Device_Connect(void);

void EVENT_USB_Device_Disconnect(void);

void EVENT_USB_Device_ConfigurationChanged(void);

void EVENT_USB_Device_ControlRequest(void);

/**
 * @}
 */

/**
 * @}
 */

#ifdef __cplusplus
}
#endif
#endif
