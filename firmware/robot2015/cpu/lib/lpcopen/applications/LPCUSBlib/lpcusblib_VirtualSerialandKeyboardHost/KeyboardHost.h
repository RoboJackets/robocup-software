/*
 * @brief Definitions and declarations of KeyBoard Host
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

#ifndef __KEYBOARD_HOST_H_
#define __KEYBOARD_HOST_H_

#include "board.h"
#include "USB.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @defgroup Virtual_Serial_Device_and_Keyboard_Host Virtual Serial Device and Keyboard Host
 * @ingroup EXAMPLES_USB_18XX43XX
 * <b>Example description</b><br>
 * This example implements a Dual Mode USB device.<br>
 * check the @ref USB_Virtual_Serial_Device_18xx43xx for Device mode.<br>
 * check the @ref Keyboard_Host_18xx43xx for Host mode.<br>
 *
 * @note By default, device run on port 0 and host run on port 1
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
 * @{
 */

/** @defgroup Keyboard_Host_Definition_for_VirtualDevice_KeyboardHost Main definitions for Keyboard Host
 * @{
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

/** @defgroup Keyboard_Host_Functions_for_VirtualDevice_KeyboardHost Functions prototypes
 * @{
 */

/**
 * @brief	Keyboard Host task
 * This function will be called repeatedly from the main, so that
 * keyboard host driver will update its state machines and receives
 * keystrokes.
 * @return	None
 */
extern void KeyboardHost(void);

/**
 * @}
 */

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* __KEYBOARD_HOST_H_ */
