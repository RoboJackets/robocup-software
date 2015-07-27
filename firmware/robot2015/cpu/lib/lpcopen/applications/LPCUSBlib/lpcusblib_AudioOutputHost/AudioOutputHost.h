/*
 * @brief Definitions and declarations of Audio Output Host example
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

#ifndef __AUDIO_OUTPUT_HOST_H_
#define __AUDIO_OUTPUT_HOST_H_

#include "board.h"
#include "USB.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @defgroup Audio_Output_Host_18xx43xx Audio Output Host
 * @ingroup EXAMPLES_USB_18XX43XX
 * <b>Example description</b><br>
 * This example implements an audio interface class host mode device that enumerates
 * an audio interface class device (USB speakers) and sends samples to the device.
 *
 * The samples are created by a simple square wave generator in a timer ISR and
 * shipped out the streaming isochronous output pipe when a button is pressed on
 * the board.
 * When the example is first run the terminal window will display:
 * Audio Output Host Demo running.
 *
 * When the USB speakers are plugged in the terminal window will display:
 * Audio Device Enumerated.
 *
 * When the button is pressed on the board the USB speakers will produce
 * a loud tone. When the button is not pressed the USB speakers will be silent.
 *
 * When the USB speakers are unplugged the terminal window will display:
 * Device Unattached.<br>
 *
 * <b>Special connection requirements</b><br>
 * For Keil 1858 and 4357 boards, the host examples requires an external power supply
 * via the PWR power connector.<br>
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

/** @defgroup Audio_Output_Host_17xx40xx Audio Output Host
 * @ingroup EXAMPLES_USB_17XX40XX
 * <b>Example description</b><br>
 * This example implements an audio interface class host mode device that enumerates
 * an audio interface class device (USB speakers) and sends samples to the device.
 *
 * The samples are created by a simple square wave generator in a timer ISR and
 * shipped out the streaming isochronous output pipe when a button is pressed on
 * the board.
 * When the example is first run the terminal window will display:
 * Audio Output Host Demo running.
 *
 * When the USB speakers are plugged in the terminal window will display:
 * Audio Device Enumerated.
 *
 * When the button is pressed on the board the USB speakers will produce
 * a loud tone. When the button is not pressed the USB speakers will be silent.
 *
 * When the USB speakers are unplugged the terminal window will display:
 * Device Unattached.<br>
 *
 * <b>Special connection requirements</b><br>
 *  - EA1788 and EA4088 Developer's Kits<br>
 *      - Open jumper JP15 near 20 pin JTAG connector<br>
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
 */

/**
 * @}
 */

/** @defgroup Audio_Output_Host_Definition Main definitions
 * @ingroup Audio_Output_Host_18xx43xx Audio_Output_Host_17xx40xx
 * @{
 */

/** ADC channel number for the microphone input. */
#define MIC_IN_ADC_CHANNEL        2

/** ADC channel MUX mask for the microphone input. */
#define MIC_IN_ADC_MUX_MASK       ADC_CHANNEL2

/** Maximum audio sample value for the microphone input. */
#define SAMPLE_MAX_RANGE          0xFFFF

/** Maximum ADC range for the microphone input. */
#define ADC_MAX_RANGE             0x3FF

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

#ifdef __cplusplus
}
#endif

#endif /* __AUDIO_OUTPUT_HOST_H_ */
