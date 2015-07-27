/*
 * @brief LPC43XX dual core common functions and defines
 *
 * @note
 * Copyright(C) NXP Semiconductors, 2012
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

#ifndef __DUALCORE_COMMON_H_
#define __DUALCORE_COMMON_H_

/** @defgroup EXAMPLES_DUALCORE_43XX_COMMON LPC43xx dual core common code
 * @ingroup EXAMPLES_DUALCORE_43XX
 * Common code shared among dual-core examples
 *
 * Build procedure:
 * <a href="http://www.lpcware.com/content/project/lpcopen-platform-nxp-lpc-microcontrollers/lpcopen-build-procedures/lpc18xx/43xx-lpco">LPCOpen 18xx/43xx build instructions</a>
 *
 * Submit bug reports for LPCOpen code <a href="http://www.lpcware.com/content/bugtrackerproject/lpcopen">here.</a>
 * @{
 */

/*
 * Select the default device to which the image
 * be loaded, based on the board. It could be overridden
 * by the user by defining one of the TARGET_XXXXX in the
 * compiler defines.
 */
#if (!defined(TARGET_SPIFI) && !defined(TARGET_IFLASH) && !defined(TARGET_XFLASH))
	#ifdef BOARD_HITEX_EVA_18504350
		#define TARGET_SPIFI
	#elif defined(BOARD_KEIL_MCB_18574357)
		#define TARGET_IFLASH
	#elif defined(BOARD_NGX_XPLORER_18304330)
		#define TARGET_SPIFI
	#else
		#error "Unknown load target!"
	#endif
#endif

/* Selecting base address based on Target */
#ifdef TARGET_SPIFI
	#define IMAGE_BASE_ADDR (SPIFI_BASE_ADDR)
#elif defined(TARGET_XFLASH)
	#define IMAGE_BASE_ADDR (XFLASH_BASE_ADDR)
#elif defined(TARGET_IFLASH)
	#define IMAGE_BASE_ADDR (IFLASH_BASE_ADDR)
#else
	#error "Unknown load target!"
#endif

/* Compiler specific attributes */
#if defined(__IAR_SYSTEMS_ICC__)
	#define LOCATE_ATX(x)      _Pragma(#x)
	#define LOCATE_ATXX(x)     LOCATE_ATX(location=x)
	#define LOCATE_AT(x)       LOCATE_ATXX(x)
	#define WEAK_SYMBOL        __weak
#elif defined(__ARMCC_VERSION)
	#define LOCATE_AT(x)     __attribute__((at(x)))
	#define WEAK_SYMBOL      __attribute__((weak))
#elif (defined(__CODE_RED))
	#define LOCATE_ATX(x)     __attribute__((section(".shmem_"#x ",\"aw\",%nobits@")))
	#define LOCATE_AT(x) LOCATE_ATX(x)
	#define WEAK_SYMBOL      __attribute__((weak))
#else
	#error "Unsupported Compiler/Tool-Chain!"
#endif

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief	IPC Example initialization
 *
 * This function initializes the IPC Example implementation. It calls
 * IPC_initMsgQueue() to initialize the IPC queue of the calling core,
 * by allocating and passing a queue array. It will be called by the
 * @link EXAMPLE_DUALCORE_CMN_MAIN main()@endlink of both core-M0 and
 * core-M4 code for all the dual core examples.
 *
 * @return	None
 */
extern void IPCEX_Init(void);

/**
 * @brief	USB Host example initialization
 *
 * Function that initializes usb host stack by calling USB_Init() and
 * the dual core USB host example. *This function is called by
 * @link EXAMPLE_DUALCORE_CMN_MAIN main()@endlink only when EXAMPLE_USB_HOST
 * is defined*.
 *
 *@return	None
 */
extern void USBHOST_Init(void);

/**
 * @brief	USB Device example initialization
 *
 * Function that initializes usb device stack by calling USB_Init() and
 * the dual core USB device example. *This function is called by
 * @link EXAMPLE_DUALCORE_CMN_MAIN main()@endlink only when EXAMPLE_USB_DEVICE
 * is defined*.
 *
 *@return	None
 */
extern void USBDEV_Init(void);

/**
 * @brief	lwIP dual core example initialisation function
 *
 * This function initializes the lwIP interface (ethernet etc.). In
 * stand-alone configuration, this function will enable the tick
 * functionality. It will initialises the LWIP stack, initialises
 * the network interface, initialises the DHCP & HTTPD functions.
 * In FreeRTOS/uCOS-III configurations, it does nothing as the
 * initialization is handed by the corresponding tasks.  *This function
 * is called by @link EXAMPLE_DUALCORE_CMN_MAIN main()@endlink only
 * when EXAMPLE_LWIP is defined*.
 *
 * @return None
 */
extern void LWIP_Init(void);

/**
 * @brief	emWin dual core example initialisation function
 *
 * The function initialises the LCD controller & Touch screen controller.
 * It will register the IPC callbacks to receive graphics information from
 * other processor core. In stand-alone configuration, it will also create
 * the emWin Widget framewindow of the graphics application. *This function
 * is called by @link EXAMPLE_DUALCORE_CMN_MAIN main()@endlink only
 * when EXAMPLE_EMWIN is defined*.
 *
 * @return None
 */
extern void EMWIN_Init(void);

/**
 * @brief	Dual core Blinky example initialization
 *
 * This function will register the blink function to the event queue of
 * a Core (M0/M4). Whenever a blink event is received from the other core
 * the registered call-back function will be called-back by the ipcex_task(),
 * *This function is called by @link EXAMPLE_DUALCORE_CMN_MAIN main()@endlink
 * only when EXAMPLE_BLINKY is defined*. In all the dual core examples
 * EXAMPLE_BLINKY is defined by default.
 *
 * @return	None
 */
extern void BLINKY_Init(void);

/**
 * @brief	Dual Core IPC example implementation task
 *
 * This task receives the message from the other core and will call-back the
 * function (if registered via ipcex_register_callback()) corresponding to
 * the received message. This function is called by @link
 * EXAMPLE_DUALCORE_CMN_MAIN main()@endlink.
 *
 * @return	None
 */
extern void ipcex_tasks(void);

/**
 * @brief	Dual core blinky task
 *
 * Calling this function calls the blinky task in which will blink the
 * LEDs. When no OS is specified this will be repeatedly *called from
 * dual core @link EXAMPLE_DUALCORE_CMN_MAIN main()@endlink or once
 * if any OS is defined, only when EXAMPLE_BLINKY is defined*. In all
 * the dual core examples EXAMPLE_BLINKY is defined by default.
 *
 * @return	None
 */
extern void blinky_tasks(void);

/**
 * @brief	Dual Core USB host task
 *
 * This function creates the task that invokes the host tasks
 * and the USB_USBTask() provided by the USB library. *This
 * function is called by @link EXAMPLE_DUALCORE_CMN_MAIN main()@endlink
 * only when EXAMPLE_USB_HOST is defined*.
 *
 * @return None
 */
extern void usb_host_tasks(void);

/**
 * @brief	Dual Core USB device task
 *
 * This function creates the task that invokes the class driver
 * task and the USB_USBTask() provided by the USB library. *This
 * function is called by @link EXAMPLE_DUALCORE_CMN_MAIN main()@endlink
 * only when EXAMPLE_USB_DEVICE is defined*.
 *
 * @return None
 */
extern void usb_device_tasks(void);

/**
 * @brief	LWIP Task function
 *
 * Function that creates/performs the lwIP stack functionality.
 * In stand-alone configuration, this function will monitor the link
 * status and handles the packets. In FreeRTOS/uCOS-III configurations,
 * it will create the network interface task. *Called by dual core
 * @link EXAMPLE_DUALCORE_CMN_MAIN main()@endlink only when
 * EXAMPLE_LWIP is defined*.
 *
 * @return None
 */
extern void lwip_tasks(void);

/**
 * @brief	emWin dual core example task
 *
 * In stand-alone configuration, it will update the fields on LCD screen. In
 * FreeRTOS/uCOS-III configurations, this will create the LCD & Touch Screen
 * application tasks. *Called by dual core @link EXAMPLE_DUALCORE_CMN_MAIN
 * main()@endlink only when EXAMPLE_EMWIN is defined*.
 *
 * @return None
 */
extern void emwin_tasks(void);

/**
 * @brief	Millisecond sleep
 *
 * Calling this function will sleep for \a msec number of milli seconds
 * by a busy wait loop. This function uses the RITIMER in LPC43XX to calculate
 * the time lapsed.
 *
 * @param	msecs	: Number of milli seconds to sleep
 * @return	None
 */
void MSleep(int32_t msecs);

/**
 * @brief	Initialize M0 and boot the core with image at \a m0_image_addr
 *
 * This function is avilable only for code that runs on M4 core, and defined in
 * m0_ImageLoader.c
 *
 * @param	m0_image_addr	: uint32_t converted pointer to M0 image address
 * @return	0 on success -1 on error
 */
int M0Image_Boot(uint32_t m0_image_addr);

#ifdef __cplusplus
}
#endif

/**
 * @}
 */

#endif
