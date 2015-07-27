/*
 * @brief	HTTP server dual core example using lwIP ethernet stack
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
 * \file webserver\dualcore_webserver.c
 * LWIP HTTP Server dual core example application source code
 * This file implements the HTTP server functionality for stand-alone,
 * FreeRTOS, and uCOS-III configurations
 */

/* General Includes */
#include <string.h>
#include "lpc43xx_dualcore_config.h"
#include "ipc_msg.h"
#include "ipc_example.h"

/* lwIP include files */
#include "lwip/opt.h"
#include "lwip/sys.h"
#include "lwip/memp.h"
#include "lwip/tcpip.h"
#include "lwip/ip_addr.h"
#include "lwip/netif.h"
#include "lwip/timers.h"

#include "netif/etharp.h"

#if LWIP_DHCP
#include "lwip/dhcp.h"
#endif

#include "arch/lpc18xx_43xx_emac.h"
#include "arch/lpc_arch.h"
#include "lpc_phy.h"/* For the PHY monitor support */

#if (defined(OS_FREE_RTOS) || defined(OS_UCOS_III))
extern void http_server_netconn_init(void);
#else
#include "httpd.h"
#endif

#ifdef OS_FREE_RTOS
#include "FreeRTOS.h"
#include "task.h"
#endif

#ifdef OS_UCOS_III
#include "os.h"

#define UCOS_MIN_STACK_SZ            512
#define UCOS_BLINK_TASK_PRIORITY     12
#define UCOS_EVENT_TASK_PRIORITY     13
#endif

/** @defgroup EXAMPLE_DUALCORE_LWIP LPC18xx/43xx HTTP Server dual core example using LWIP ethernet stack
 * @ingroup EXAMPLES_DUALCORE_43XX
 * <b>Example description</b><br>
 * The LWIP HTTP Server example demonstrates the HTTP Server example using LWIP ethernet stack.
 * The HTTP server can be configured to run on any of the cores (M4 or M0).
 * The user should connect the board to the network by using the ethernet cable. The board will
 * get an IP address by using DHCP method. The use can access the HTTP Server by using a
 * web browser from the host PC.<br>
 * If the USB Mass storage is compiled in the application (on either M0/M4 core), then it will
 * read the HTTP contents from USB Mass storage disk & provided to the user.
 * If the USB Mass storage is not compiled in the application (on either M0/M4 core), then the
 * default HTTP page will be displayed.<br>
 * In FreeRTOS/uCOS-III configurations, the net_conn API interface will be used.
 * In stand-alone configuration, HTTPD interface will be used.<br>
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

/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/

/* LWIP Network interface object structure */
static struct netif lpc_netif;

/*****************************************************************************
 * Private functions
 ****************************************************************************/

static void ip_addr_changed(const struct netif *nwif)
{
	char tmp_buff[16];
	extern void EVENT_lwip_addr_changed(uint32_t);

	if (!nwif->ip_addr.addr) return;

	DEBUGOUT("IP_ADDR    : %s\r\n", ipaddr_ntoa_r((const ip_addr_t *) &nwif->ip_addr, tmp_buff, 16));
	DEBUGOUT("NET_MASK   : %s\r\n", ipaddr_ntoa_r((const ip_addr_t *) &nwif->netmask, tmp_buff, 16));
	DEBUGOUT("GATEWAY_IP : %s\r\n", ipaddr_ntoa_r((const ip_addr_t *) &nwif->gw, tmp_buff, 16));
	EVENT_lwip_addr_changed(nwif->ip_addr.addr);
}

#if (defined(OS_FREE_RTOS) || defined(OS_UCOS_III))

/**
 * Callback function for TCPIP thread to indicate TCPIP init is done
 * (Only used in case of FreeRTOS/uCOS-III configuration)
 */
static void tcpip_init_done_signal(void *arg)
{
	/* Tell main thread TCP/IP init is done */
	*(s32_t *) arg = 1;
}

/**
 * Network interface setup function
 * (Only used in case of FreeRTOS/uCOS-III configuration)
 */
#ifdef OS_FREE_RTOS
static portTASK_FUNCTION(vSetupIFTask, pvParameters)
#elif  OS_UCOS_III
static void lwip_app_task(void *arg)
#endif
{
	ip_addr_t ipaddr, netmask, gw;
	volatile s32_t tcpipdone = 0;
	uint32_t physts;
	static int prt_ip = 0;

	/* Wait until the TCP/IP thread is finished before
	   continuing or wierd things may happen */
	DEBUGSTR("Waiting for TCPIP thread to initialize...\r\n");
	tcpip_init(tcpip_init_done_signal, (void *) &tcpipdone);
	while (!tcpipdone) ;

	DEBUGSTR("Starting LWIP HTTP server...\r\n");

	/* Static IP assignment */
#if LWIP_DHCP
	IP4_ADDR(&gw, 0, 0, 0, 0);
	IP4_ADDR(&ipaddr, 0, 0, 0, 0);
	IP4_ADDR(&netmask, 0, 0, 0, 0);
#else
	IP4_ADDR(&gw, 10, 1, 10, 1);
	IP4_ADDR(&ipaddr, 10, 1, 10, 234);
	IP4_ADDR(&netmask, 255, 255, 255, 0);
#endif

	/* Add netif interface for lpc17xx_8x */
	memset(&lpc_netif, 0, sizeof(lpc_netif));
	if (!netif_add(&lpc_netif, &ipaddr, &netmask, &gw, NULL, lpc_enetif_init,
				   tcpip_input)) {
		DEBUGSTR("Net interface failed to initialize ..\r\n");
		while (1) ;
	}

	netif_set_default(&lpc_netif);
	netif_set_up(&lpc_netif);

	/* Enable MAC interrupts only after LWIP is ready */
	NVIC_SetPriority(ETHERNET_IRQn, IRQ_PRIO_ETHERNET);
	NVIC_EnableIRQ(ETHERNET_IRQn);

#if LWIP_DHCP
	dhcp_start(&lpc_netif);
#endif

	/* Initialize and start application */
	http_server_netconn_init();

	/* This loop monitors the PHY link and will handle cable events
	   via the PHY driver. */
	while (1) {
		/* Call the PHY status update state machine once in a while
		   to keep the link status up-to-date */
		physts = lpcPHYStsPoll();

		/* Only check for connection state when the PHY status has changed */
		if (physts & PHY_LINK_CHANGED) {
			if (physts & PHY_LINK_CONNECTED) {
				/* Set interface speed and duplex */
				if (physts & PHY_LINK_SPEED100) {
					NETIF_INIT_SNMP(&lpc_netif, snmp_ifType_ethernet_csmacd, 100000000);
				}
				else {
					NETIF_INIT_SNMP(&lpc_netif, snmp_ifType_ethernet_csmacd, 10000000);
				}
				if (physts & PHY_LINK_FULLDUPLX) {
					Chip_ENET_SetDuplex(LPC_ETHERNET, true);
				}
				else {
					Chip_ENET_SetDuplex(LPC_ETHERNET, false);
				}

				tcpip_callback_with_block((tcpip_callback_fn) netif_set_link_up,
										  (void *) &lpc_netif, 1);
			}
			else {
				tcpip_callback_with_block((tcpip_callback_fn) netif_set_link_down,
										  (void *) &lpc_netif, 1);
			}
		}

		// DEBUGOUT("Link connect status: %d\n", ((physts & PHY_LINK_CONNECTED) != 0));

		/* Delay for link detection */
		msDelay(250);

		if (!prt_ip && lpc_netif.ip_addr.addr) {
			prt_ip = 1;
			ip_addr_changed(&lpc_netif);
		}
	}
}

#endif

/*****************************************************************************
 * Public functions
 ****************************************************************************/

extern void lcd_update_hostip(uint32_t host_ip);

#ifdef OS_FREE_RTOS
/**
 * @brief	Delay function
 * @param ms	:   Delay value in milliseconds
 * @return None
 * The function provides the delay (Only used in FreeRTOS configuration)
 */
/*********************************************************************//**
 * @brief  MilliSecond delay function based on FreeRTOS
 * @param[in] ms Number of milliSeconds to delay
 * @note		Needed for some functions, do not use prior to FreeRTOS running
 * @return Nothing
 **********************************************************************/
void msDelay(uint32_t ms)
{
	vTaskDelay((configTICK_RATE_HZ * ms) / 1000);
}

#endif

/* lwIP initialization function */
void LWIP_Init(void)
{
	#ifdef OS_FREE_RTOS
	return;
	#elif defined(OS_UCOS_III)
	return;
	#else
	extern void lwip_init(void);

	ip_addr_t ipaddr, netmask, gw;

	/* Setup a 1mS sysTick for the primary time base */
	SysTick_Enable(1);

	/* Initialize LWIP */
	DEBUGSTR("Starting lwip_init...\r\n");
	lwip_init();

	DEBUGSTR("Starting LWIP HTTP server...\r\n");

	/* Static IP assignment */
#if LWIP_DHCP
	IP4_ADDR(&gw, 0, 0, 0, 0);
	IP4_ADDR(&ipaddr, 0, 0, 0, 0);
	IP4_ADDR(&netmask, 0, 0, 0, 0);
#else
	IP4_ADDR(&gw, 10, 1, 10, 1);
	IP4_ADDR(&ipaddr, 10, 1, 10, 234);
	IP4_ADDR(&netmask, 255, 255, 255, 0);
#endif

	/* Add netif interface for lpc17xx_8x */
	netif_add(&lpc_netif, &ipaddr, &netmask, &gw, NULL, lpc_enetif_init,
			  ethernet_input);
	netif_set_default(&lpc_netif);
	netif_set_up(&lpc_netif);

#if LWIP_DHCP
	dhcp_start(&lpc_netif);
#endif

	/* Initialize and start application */
	httpd_init();

	return;
#endif
}

#if (defined(OS_FREE_RTOS) || defined(OS_UCOS_III))
/**
 * @brief	LWIP Task function
 * @return None
 * The function is called from the main task  function.
 * In stand-alone configuration, this function will monitor the link status and
 * handles the packets.
 * In FreeRTOS/uCOS-III configurations, it will create the network interface task
 */
void lwip_tasks(void)
{
#ifdef OS_FREE_RTOS
	xTaskCreate(vSetupIFTask, (signed char *) "SetupIFx",
				256, NULL, (tskIDLE_PRIORITY + 1UL),
				(xTaskHandle *) NULL);
#elif OS_UCOS_III
	OS_ERR os_err;
	static OS_TCB lwip_app_taskTCB;
	static CPU_STK lwip_app_taskSTK[UCOS_MIN_STACK_SZ];

	OSTaskCreate((OS_TCB      *) &lwip_app_taskTCB,				/* Create the start task                                */
				 (CPU_CHAR    *) "Start",
				 (OS_TASK_PTR) lwip_app_task,
				 (void        *) 0,
				 (OS_PRIO) (APP_CFG_TASK_START_PRIO),
				 (CPU_STK     *) &lwip_app_taskSTK[0],
				 (CPU_STK_SIZE) 32,
				 (CPU_STK_SIZE) UCOS_MIN_STACK_SZ,
				 (OS_MSG_QTY) 0u,
				 (OS_TICK) 0u,
				 (void        *) 0,
				 (OS_OPT) (OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
				 (OS_ERR      *) &os_err);
#endif
}

#else

/* lwIP Tasks */
void lwip_tasks(void)
{
	uint32_t physts;
	static int prt_ip = 0;

	do {
		/* Handle packets as part of this loop, not in the IRQ handler */
		lpc_enetif_input(&lpc_netif);

		/* Free TX buffers that are done sending */
		lpc_tx_reclaim(&lpc_netif);

		/* LWIP timers - ARP, DHCP, TCP, etc. */
		sys_check_timeouts();

		/* Call the PHY status update state machine once in a while
		   to keep the link status up-to-date */
		physts = lpcPHYStsPoll();

		/* Only check for connection state when the PHY status has changed */
		if (physts & PHY_LINK_CHANGED) {
			if (physts & PHY_LINK_CONNECTED) {
				/* Set interface speed and duplex */
				if (physts & PHY_LINK_SPEED100) {
					NETIF_INIT_SNMP(&lpc_netif, snmp_ifType_ethernet_csmacd, 100000000);
				}
				else {
					NETIF_INIT_SNMP(&lpc_netif, snmp_ifType_ethernet_csmacd, 10000000);
				}
				if (physts & PHY_LINK_FULLDUPLX) {
					Chip_ENET_SetDuplex(LPC_ETHERNET, true);
				}
				else {
					Chip_ENET_SetDuplex(LPC_ETHERNET, false);
				}

				netif_set_link_up(&lpc_netif);
			}
			else {
				netif_set_link_down(&lpc_netif);
			}
		}

		/* DEBUGOUT("Link connect status: %d\r\n", ((physts & PHY_LINK_CONNECTED) != 0)); */

		if (!prt_ip && lpc_netif.ip_addr.addr) {
			prt_ip = 1;
			ip_addr_changed(&lpc_netif);
		}
	} while (0);
}

#endif

#ifdef __IAR_SYSTEMS_ICC__
WEAK_SYMBOL
#else
WEAK_SYMBOL void EVENT_lwip_addr_changed(uint32_t new_addr);
#endif

/**
 * \brief	Event handler to notify IP address change to other tasks
 * This is a weak function implementation, other tasks/applicaitons
 * can override the function with its own implementation
 * \param	new_addr	: New IP address obtained from DHCP
 * \return	None
 */
void EVENT_lwip_addr_changed(uint32_t new_addr)
{
	ipcex_msgPush(IPCEX_ID_USER1, new_addr);
}
/**
 * @}
 */
