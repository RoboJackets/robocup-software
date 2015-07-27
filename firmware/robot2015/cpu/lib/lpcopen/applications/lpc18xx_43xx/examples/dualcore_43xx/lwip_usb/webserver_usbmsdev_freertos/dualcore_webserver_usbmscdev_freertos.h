/*
 * @brief	LWIP HTTP server & LPCUSBlib Mass Storage Device dual core example FreeRTOS version
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

/**
 * \file webserver_usbmsdev_freertos\dualcore_webserver_usbmscdev_freertos.h
 * Dummy file
 */

/** @defgroup EXAMPLE_DUALCORE_LWIP_MassStorage_FreeRTOS LWIP HTTP Server & LPCUSBlib USB Mass Storage Device dual core example FreeRTOS version
 * @ingroup EXAMPLES_DUALCORE_43XX
 * The LWIP HTTP Server is implemented using LWIP ethernet stack & USB Mass Storage functionality is
 * implemented using LCPUSBlib library with FreeRTOS.
 * The HTTP server & USB Mass storage functionality can be configured to run on any of the cores (M4 or M0).
 * The user should connect the board to the network by using the ethernet cable. The board will
 * get an IP address by using DHCP method. The use can access the HTTP Server by using a
 * web browser from the host PC.
 * The HTTP contents from USB Mass storage disk will be read & provided to the user.
 * In FreeRTOS/uCOS-III configurations, the net_conn API interface will be used.
 * In stand-alone configuration, HTTPD interface will be used.
 * Please note that this application includes the code in "LPCUSBlib Mass Storage Device Dual core example" &
 * "HTTP Server dual core example using LWIP ethernet stack" stand-alone applications.
 * Please refer to the documentation in these examples.
 *
 * @{
 */

/**
 * @}
 */
