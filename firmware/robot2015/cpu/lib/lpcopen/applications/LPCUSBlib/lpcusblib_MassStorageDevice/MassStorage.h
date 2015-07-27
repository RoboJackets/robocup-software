/*
 * @brief Definitions and declarations of Mass Storage device example
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

#ifndef __MASS_STORAGE_H_
#define __MASS_STORAGE_H_

#include "board.h"
#include "USB.h"
#include <string.h>
#include "Descriptors.h"
#include "Lib/SCSI.h"
#include "Lib/DataRam.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @defgroup USB_Mass_Storage_Device_18xx43xx Mass Storage Device
 * @ingroup EXAMPLES_USB_18XX43XX
 * <b>Example description</b><br>
 * The Mass Storage Device example demonstrates a USB Mass Storage Device
 * using LPCUSBlib library. The MCU will be enumerated as as Mass Storage Device when
 * connected to the Host PC. When **CFG_SDCARD** is not defined then the example uses
 * the RAM to simulate a small massstorage device.<br>
 *
 * When **CFG_SDCARD** is defined then the board will enumerate the SD CARD connected
 * to the SD card slot as the USB mass storage device. To get the SD CARD working the
 * following board setup must be made.<br>
 *
 * <b>Special connection requirements</b><br>
 * When **CFG_SDCARD** not defined:<br>
 *  - Hitex LPC1850EVA-A4-2 and LPC4350EVA-A4-2 boards<br>
 *     - Close pin 1-2 of JP9<br>
 *     - Open all SV3, SV6<br>
 *     - Close all SV12<br>
 *     - Uart1 terminal port: SV11-p2(U1_RXD) connect to SV1-p7, SV11-p4 (U1_TXD) connect to SV1-p5<br>
 *     - Optional: Debug UART must be changed to LPC_UART1 from LPC_USART0 (in sys_config.h)<br>
 *  - Keil MCB1857 and MCB4357 boards (debug output terminal on Uart3)<br>
 *  - NGX LPC1830 LPC4330 (no debug output terminal)<br>
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

/**
 * @}
 */

/** @defgroup USB_Mass_Storage_Device_17xx40xx Mass Storage Device
 * @ingroup EXAMPLES_USB_17XX40XX
 * <b>Example description</b><br>
 * The Mass Storage Device example demonstrates a USB Mass Storage Device
 * using LPCUSBlib library. The MCU will be enumerated as as Mass Storage Device when
 * connected to the Host PC. When **CFG_SDCARD** is not defined then the example uses
 * the RAM to simulate a small massstorage device.<br>
 *
 * When **CFG_SDCARD** is defined then the board will enumerate the SD CARD connected
 * to the SD card slot as the USB mass storage device. To get the SD CARD working the
 * following board setup must be made.<br>
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

/** @defgroup USB_Mass_Storage_Device_13xx Mass Storage Device
 * @ingroup EXAMPLES_USB_13XX
 * <b>Example description</b><br>
 * The Mass Storage Device example demonstrates a USB Mass Storage Device
 * using LPCUSBlib library. The MCU will be enumerated as as Mass Storage Device when
 * connected to the Host PC. When **CFG_SDCARD** is not defined then the example uses
 * the RAM to simulate a small massstorage device.<br>
 *
 * When **CFG_SDCARD** is defined then the board will enumerate the SD CARD connected
 * to the SD card slot as the USB mass storage device. To get the SD CARD working the
 * following board setup must be made.<br>
 *
 * <b>Special connection requirements</b><br>
 * There are no special connection requirements for this example.<br>
 *
 * <b>Build procedures:</b><br>
 * @ref LPCOPEN_13XX_BUILDPROCS_XPRESSO<br>
 * @ref LPCOPEN_13XX_BUILDPROCS_KEIL<br>
 * @ref LPCOPEN_13XX_BUILDPROCS_IAR<br>
 *
 * <b>Supported boards and board setup:</b><br>
 * @ref LPCOPEN_13XX_BOARD_XPRESSO_1347<br>
 *
 * <b>Submitting LPCOpen issues:</b><br>
 * @ref LPCOPEN_COMMUNITY
 * @{
 */

/**
 * @}
 */

/** @defgroup USB_Mass_Storage_Device_11Uxx Mass Storage Device
 * @ingroup EXAMPLES_USB_11UXX
 * <b>Example description</b><br>
 * The Mass Storage Device example demonstrates a USB Mass Storage Device
 * using LPCUSBlib library. The MCU will be enumerated as as Mass Storage Device when
 * connected to the Host PC. When **CFG_SDCARD** is not defined then the example uses
 * the RAM to simulate a small massstorage device.<br>
 *
 * When **CFG_SDCARD** is defined then the board will enumerate the SD CARD connected
 * to the SD card slot as the USB mass storage device. To get the SD CARD working the
 * following board setup must be made.<br>
 *
 * <b>Special connection requirements</b><br>
 * There are no special connection requirements for this example.<br>
 *
 * <b>Build procedures:</b><br>
 * @ref LPCOPEN_11XX_BUILDPROCS_XPRESSO<br>
 * @ref LPCOPEN_11XX_BUILDPROCS_KEIL<br>
 * @ref LPCOPEN_11XX_BUILDPROCS_IAR<br>
 *
 * <b>Supported boards and board setup:</b><br>
 * @ref LPCOPEN_11XX_BOARD_XPRESSO_11U14<br>
 *
 * <b>Submitting LPCOpen issues:</b><br>
 * @ref LPCOPEN_COMMUNITY
 * @{
 */

/**
 * @}
 */

/** @defgroup Mass_Storage_Device_Definition Main definitions
 * @ingroup USB_Mass_Storage_Device_18xx43xx USB_Mass_Storage_Device_17xx40xx USB_Mass_Storage_Device_11Uxx
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

/** LED mask for the library LED driver, to indicate that the USB interface is busy. */
#define LEDMASK_USB_BUSY          LEDS_LED2

/** Total number of logical drives within the device - must be non-zero. */
#define TOTAL_LUNS                1

/*		#define VIRTUAL_MEMORY_BYTES                (MassStorage_GetCapacity()) */
/*		#define VIRTUAL_MEMORY_BLOCK_SIZE           512 */
/*		#define VIRTUAL_MEMORY_BLOCKS               (VIRTUAL_MEMORY_BYTES / VIRTUAL_MEMORY_BLOCK_SIZE) */
/** Blocks in each LUN, calculated from the total capacity divided by the total number of Logical Units in the device. */
#define LUN_MEDIA_BLOCKS         (VIRTUAL_MEMORY_BLOCKS / TOTAL_LUNS)

/** Indicates if the disk is write protected or not. */
#define DISK_READ_ONLY            false

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* __MASS_STORAGE_H_ */
