/*
 * @brief LPC8xx basic chip inclusion file
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

#ifndef __CHIP_H_
#define __CHIP_H_

#include "lpc_types.h"
#include "sys_config.h"
#include "cmsis.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifndef CORE_M0PLUS
#error CORE_M0PLUS is not defined for the LPC8xx architecture
#error CORE_M0PLUS should be defined as part of your compiler define list
#endif

#ifndef CHIP_LPC8XX
#error The LPC8XX Chip include path is used for this build, but
#error CHIP_LPC8XX is not defined!
#endif

/** @defgroup IP_LPC8XX_FILES CHIP: LPC8XX Chip layer required IP layer drivers
 * @ingroup CHIP_8XX_Drivers
 * This is a list of the IP drivers required for the LPC8XX device family.<br>
 * (acmp_001.c, acmp_001.h) @ref IP_ACMP_001<br>
 * (crc_001.c, crc_001.h) @ref IP_CRC_001<br>
 * (gpio_001.h) @ref IP_GPIO_001<br>
 * (mrt_001.h) @ref IP_MRT_001<br>
 * (sct_001.c, sct_001.h) @ref IP_SCT_001<br>
 * (spi_002.c, spi_002.h) @ref IP_SPI_002<br>
 * (usart_002.c, usart_002.h) @ref IP_USART_002<br>
 * (wkt_001.h) @ref IP_WKT_001<br>
 * (wwdt_001.c, wwdt_001.h) @ref IP_WWDT_001<br>
 * @{
 */

/**
 * @}
 */

#include "acmp_001.h"
#include "crc_001.h"
#include "gpio_001.h"
#include "mrt_001.h"
#include "spi_002.h"
#include "usart_002.h"
#include "wwdt_001.h"
#include "sct_001.h"
#include "wkt_001.h"

/** @defgroup PERIPH_8XX_BASE CHIP: LPC8xx Peripheral addresses and register set declarations
 * @ingroup CHIP_8XX_Drivers
 * @{
 */

/* Base addresses */
#define LPC_FLASH_BASE        (0x00000000UL)
#define LPC_RAM_BASE          (0x10000000UL)
#define LPC_ROM_BASE          (0x1FFF0000UL)
#define LPC_APB0_BASE         (0x40000000UL)
#define LPC_AHB_BASE          (0x50000000UL)

/* APB0 peripherals */
#define LPC_WWDT_BASE         (0x40000000UL)
#define LPC_MRT_BASE          (0x40004000UL)
#define LPC_WKT_BASE          (0x40008000UL)
#define LPC_SWM_BASE          (0x4000C000UL)
#define LPC_PMU_BASE          (0x40020000UL)
#define LPC_CMP_BASE          (0x40024000UL)

#define LPC_FMC_BASE          (0x40040000UL)
#define LPC_IOCON_BASE        (0x40044000UL)
#define LPC_SYSCTL_BASE       (0x40048000UL)
#define LPC_I2C_BASE          (0x40050000UL)
#define LPC_SPI0_BASE         (0x40058000UL)
#define LPC_SPI1_BASE         (0x4005C000UL)
#define LPC_USART0_BASE       (0x40064000UL)
#define LPC_USART1_BASE       (0x40068000UL)
#define LPC_USART2_BASE       (0x4006C000UL)

/* AHB peripherals */
#define LPC_CRC_BASE          (0x50000000UL)
#define LPC_SCT_BASE          (0x50004000UL)

#define LPC_GPIO_PORT_BASE    (0xA0000000UL)
#define LPC_PIN_INT_BASE      (0xA0004000UL)

/* Normalize types */
typedef IP_SPI_002_T  	LPC_SPI_T;
typedef IP_WWDT_001_T 	LPC_WWDT_T;
typedef IP_USART_002_T 	LPC_USART_T;
typedef IP_CRC_001_T 	LPC_CRC_T;
typedef IP_SCT_001_T 	LPC_SCT_T;
typedef ACMP_001_T 	LPC_CMP_T;
typedef IP_WKT_001_T 	LPC_WKT_T;
typedef IP_GPIO_001_T 	LPC_GPIO_T;

#define LPC_WWDT            ((LPC_WWDT_T     *) LPC_WWDT_BASE)
#define LPC_SPI0            ((LPC_SPI_T      *) LPC_SPI0_BASE)
#define LPC_SPI1            ((LPC_SPI_T      *) LPC_SPI1_BASE)
#define LPC_USART0          ((LPC_USART_T    *) LPC_USART0_BASE)
#define LPC_USART1          ((LPC_USART_T    *) LPC_USART1_BASE)
#define LPC_USART2          ((LPC_USART_T    *) LPC_USART2_BASE)
#define LPC_WKT             ((LPC_WKT_T         *) LPC_WKT_BASE)
#define LPC_PMU             ((LPC_PMU_T         *) LPC_PMU_BASE)
#define LPC_CRC             ((LPC_CRC_T         *) LPC_CRC_BASE)
#define LPC_SCT             ((LPC_SCT_T         *) LPC_SCT_BASE)
#define LPC_GPIO_PORT       ((LPC_GPIO_T        *) LPC_GPIO_PORT_BASE)
#define LPC_IOCON           ((LPC_IOCON_T       *) LPC_IOCON_BASE)
#define LPC_SWM             ((LPC_SWM_T         *) LPC_SWM_BASE)
#define LPC_SYSCTL          ((LPC_SYSCTL_T      *) LPC_SYSCTL_BASE)
#define LPC_CMP             ((LPC_CMP_T         *) LPC_CMP_BASE)
#define LPC_FMC             ((LPC_FMC_T         *) LPC_FMC_BASE)
#define LPC_MRT  			((LPC_MRT_T         *) LPC_MRT_BASE)

/**
 * @}
 */

#include "iocon_8xx.h"
#include "i2c_8xx.h"
#include "syscon_8xx.h"
#include "clock_8xx.h"
#include "swm_8xx.h"
#include "acmp_8xx.h"
#include "crc_8xx.h"
#include "pinint_8xx.h"
#include "gpio_8xx.h"
#include "uart_8xx.h"
#include "wwdt_8xx.h"
#include "spi_8xx.h"
#include "uartd_8xx.h"
#include "pwrd_8xx.h"
#include "i2cd_8xx.h"
#include "romapi_8xx.h"
#include "wkt_8xx.h"
#include "pmu_8xx.h"
#include "mrt_8xx.h"
#include "fmc_8xx.h"
#include "sct_8xx.h"

#ifdef __cplusplus
}
#endif

#endif /* __CHIP_H_ */
