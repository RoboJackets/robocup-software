/*
 * @brief Definitions and declarations of Audio Input device example
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

#ifndef __AUDIO_INPUT_H_
#define __AUDIO_INPUT_H_

#include "board.h"
#include "USB.h"
#include <stdlib.h>
#include "Descriptors.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @defgroup USB_Audio_Input_Device_18xx43xx Audio input device example
 * @ingroup EXAMPLES_USB_18XX43XX
 * <b>Example description</b><br>
 * This example implements an audio interface class device mode device that
 * enumerates as audio device (USB microphone) and sends samples to the host
 * when a button is pressed.
 * To run the example follow the steps below<br>
 * 1. Make sure USB cable is connected between the board and host PC<br>
 * 2. In host PC Press (Windows Key+R) to bring up the "Run" dialog<br>
 * 3. Enter *mmsys.cpl* in the edit box and press enter<br>
 * 4. Dialog box named *Sound* opens now, click on *Recording* tab<br>
 * 5. Among the list of audio devices shown look for *Microphone* with
 * description *LPCUSBLib Audio input Demo*<br>
 * 6. Double click the microphone entry mentioned above<br>
 * 7. *Microphone Properties* dialog will show up, in that click on *Listen* tab<br>
 * 8. Check the *Listen to this device* check box then click on @a Apply button<br>
 * 9. Press the button on the board to get the audio played on host PC<br>
 * <blockquote> If the entry mentioned in step 5 is not found, then from "Run"
 * execute command "devmgmt.msc" from the list of hardware look for the COM port
 * driver with an exclamatory mark in it. Right click and uninstall the driver, click
 * @a Action menu and select *Scan for hardware changes*. Now under "Sound Vidio and
 * Game controllers", a device named *"LPCUSBlib Audio Input Demo"* must be shown.</blockquote>
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
 */

/**
 * @}
 */

/** @defgroup USB_Audio_Input_Device_17xx40xx Audio input device example
 * @ingroup EXAMPLES_USB_17XX40XX
 * <b>Example description</b><br>
 * This example implements an audio interface class device mode device that
 * enumerates as audio device (USB microphone) and sends samples to the host
 * when a button is pressed.
 * To run the example follow the steps below<br>
 * 1. Make sure USB cable is connected between the board and host PC<br>
 * 2. In host PC Press (Windows Key+R) to bring up the "Run" dialog<br>
 * 3. Enter *mmsys.cpl* in the edit box and press enter<br>
 * 4. Dialog box named *Sound* opens now, click on *Recording* tab<br>
 * 5. Among the list of audio devices shown look for *Microphone* with
 * description *LPCUSBLib Audio input Demo*<br>
 * 6. Double click the microphone entry mentioned above<br>
 * 7. *Microphone Properties* dialog will show up, in that click on *Listen* tab<br>
 * 8. Check the *Listen to this device* check box then click on @a Apply button<br>
 * 9. Press the button on the board to get the audio played on host PC<br>
 * <blockquote> If the entry mentioned in step 5 is not found, then from "Run"
 * execute command "devmgmt.msc" from the list of hardware look for the COM port
 * driver with an exclamatory mark in it. Right click and uninstall the driver, click
 * @a Action menu and select *Scan for hardware changes*. Now under "Sound Vidio and
 * Game controllers", a device named *"LPCUSBlib Audio Input Demo"* must be shown.</blockquote>
 *
 * <b>Special connection requirements</b><br>
 *  - EA1788 and EA4088 Developer's Kits<br>
 *      - Short jumper JP15 near 20 pin JTAG connector<br>
 *
 * <b>Build procedures:</b><br>
 * @ref LPCOPEN_17XX40XX_BUILDPROCS_KEIL<br>
 * @ref LPCOPEN_17XX40XX_BUILDPROCS_IAR<br>
 * @ref LPCOPEN_17XX40XX_BUILDPROCS_XPRESSO<br>
 *
 * <b>Supported boards and board setup:</b><br>
 * @ref LPCOPEN_17XX40XX_BOARD_EA1788<br>
 * @ref LPCOPEN_17XX40XX_BOARD_EA4088<br>
 *
 * <b>Submitting LPCOpen issues:</b><br>
 * @ref LPCOPEN_COMMUNITY
 * @{
 */

/**
 * @}
 */

/** @defgroup Audio_Input_Device_Definition Main definitions
 * @ingroup USB_Audio_Input_Device_18xx43xx USB_Audio_Input_Device_17xx40xx
 * @{
 */

/**
 * @brief ADC definitions
 */
#define MIC_IN_ADC_CHANNEL        2					/*!< ADC channel number for the microphone input */
#define MIC_IN_ADC_MUX_MASK       ADC_CHANNEL2		/*!< ADC channel MUX mask for the microphone input */
#define SAMPLE_MAX_RANGE          0xFFFF			/*!< Maximum audio sample value for the microphone input */
#define ADC_MAX_RANGE             0x3FF				/*!< Maximum ADC range for the microphone input */

/**
 * @brief LED definitions
 */
#define LEDMASK_USB_NOTREADY      LEDS_LED1			/*!< LED mask for the library LED driver,
													   to indicate that the USB interface is not ready */
#define LEDMASK_USB_ENUMERATING  (LEDS_LED2 | LEDS_LED3)	/*!< LED mask for the library LED driver,
															   to indicate that the USB interface is enumerating */
#define LEDMASK_USB_READY        (LEDS_LED2 | LEDS_LED4)	/*!< LED mask for the library LED driver,
															   to indicate that the USB interface is ready */
#define LEDMASK_USB_ERROR        (LEDS_LED1 | LEDS_LED3)	/*!< LED mask for the library LED driver,
															   to indicate that an error has occurred in the USB interface */
/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* __AUDIO_INPUT_H_ */
