/*
 * @brief LPC13xx basic chip inclusion file
 *
 * Copyright(C) NXP Semiconductors, 2012
 * All rights reserved.
 *
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

#ifndef CORE_M3
#error CORE_M3 is not defined for the LPC13xx architecture
#error CORE_M3 should be defined as part of your compiler define list
#endif

#if !defined(CHIP_LPC1343) && !defined(CHIP_LPC1347)
#error CHIP_LPC1343 or CHIP_LPC1347 is not defined!
#endif

#if !defined (ENABLE_UNTESTED_CODE)
#if defined(CHIP_LPC1343)
#error The LPC1343 code has not been tested with a platform. This code should \
build without errors but may not work correctly for the device. To disable this \
error message, define ENABLE_UNTESTED_CODE.
#endif
#endif

/** @defgroup IP_LPC1343_FILES CHIP: LPC1343 Chip layer required IP layer drivers
 * @ingroup CHIP_13XX_Drivers
 * This is a list of the IP drivers required for the LPC1343 device family.<br>
 * (adc_001.c, adc_001.h) @ref IP_ADC_001<br>
 * (gpio_003.c, gpio_003.h) @ref IP_GPIO_003<br>
 * (i2c_001.c, i2c_001.h) @ref IP_I2C_001<br>
 * (ssp_001.c, ssp_001.h) @ref IP_SSP_001<br>
 * (timer_001.c, timer_001.h) @ref IP_TIMER_001<br>
 * (usart_004.c, usart_004.h) @ref IP_USART_004<br>
 * (wwdt_001.c, wwdt_001.h) @ref IP_WWDT_001<br>
 * @{
 */

/**
 * @}
 */

/** @defgroup IP_LPC1347_FILES CHIP: LPC1347 Chip layer required IP layer drivers
 * @ingroup CHIP_13XX_Drivers
 * This is a list of the IP drives required for the LPC1347 device family.<br>
 * (adc_001.c, adc_001.h) @ref IP_ADC_001<br>
 * (gpio_001.h) @ref IP_GPIO_001<br>
 * (gpiopinint_001.c, gpiopinint_001.h) @ref IP_GPIOPININT_001<br>
 * (gpiogrpint_001.c, gpiogrpint_001.h) @ref IP_GPIOGRPINT_001<br>
 * (flash_001.h) @ref IP_FLASH_EEPROM_001<br>
 * (i2c_001.c, i2c_001.h) @ref IP_I2C_001<br>
 * (ritimer_001.c, ritimer_001.h) @ref IP_RITIMER_001<br>
 * (ssp_001.c, ssp_001.h) @ref IP_SSP_001<br>
 * (timer_001.c, timer_001.h) @ref IP_TIMER_001<br>
 * (usart_004.c, usart_004.h) @ref IP_USART_004<br>
 * (wwdt_001.c, wwdt_001.h) @ref IP_WWDT_001<br>
 * @{
 */

/**
 * @}
 */

/* Peripheral mapping per device
   Peripheral					Device(s)
   ----------------------------	------------------------------------------------------------------------------
   I2C(40000000)				CHIP_LPC1343/CHIP_LPC1347
   WDT(40004000)				CHIP_LPC1343/CHIP_LPC1347
   TIMER0_16(4000C000)			CHIP_LPC1343/CHIP_LPC1347
   TIMER1_16(40010000)			CHIP_LPC1343/CHIP_LPC1347
   TIMER0_32(40014000)			CHIP_LPC1343/CHIP_LPC1347
   TIMER1_32(40018000)			CHIP_LPC1343/CHIP_LPC1347
   PMU(40038000)				CHIP_LPC1343/CHIP_LPC1347
   FLASH_CTRL(4003C000)			CHIP_LPC1343/CHIP_LPC1347
   SSP0(40040000)				CHIP_LPC1343/CHIP_LPC1347
   IOCONF(40044000)				CHIP_LPC1343/CHIP_LPC1347
   SYSCTL(40048000)				CHIP_LPC1343/CHIP_LPC1347
   SSP1(40058000)				CHIP_LPC1347
   ADC(4001C000)				CHIP_LPC1343/CHIP_LPC1347
   UART(40008000)				CHIP_LPC1343 only
   USB(40020000)				CHIP_LPC1343 only
   GPIO_PORT(50000000)			CHIP_LPC1343 only
   GPIO_PIO0(50000000)			CHIP_LPC1343 only
   GPIO_PIO1(50010000)			CHIP_LPC1343 only
   GPIO_PIO2(50020000)			CHIP_LPC1343 only
   GPIO_PIO3(50030000)			CHIP_LPC1343 only
   USART/SMARTCARD(40008000)	CHIP_LPC1347 only
   FLASH_EEPROM(4003C000)		CHIP_LPC1347 only
   GPIO_INT(4004C000)			CHIP_LPC1347 only
   GPIO_GRP0_INT(4005C000)		CHIP_LPC1347 only
   GPIO_GRP1_INT(40060000)		CHIP_LPC1347 only
   RITIMER(40064000)			CHIP_LPC1347 only
 */

#include "adc_001.h"
#include "i2c_001.h"
#include "wwdt_001.h"
#include "timer_001.h"
#include "ssp_001.h"
#include "usart_004.h"
#if defined(CHIP_LPC1343)
#include "gpio_003.h"
#else
#include "gpio_001.h"
#include "gpiogrpint_001.h"
#include "gpiopinint_001.h"
#include "ritimer_001.h"
#include "flash_001.h"
#endif

/** @defgroup PERIPH_13XX_BASE CHIP: LPC13xx Peripheral addresses and register set declarations
 * @ingroup CHIP_13XX_Drivers
 * @{
 */

#define LPC_I2C_BASE              0x40000000
#define LPC_WWDT_BASE             0x40004000
#define LPC_USART_BASE            0x40008000
#define LPC_TIMER16_0_BASE        0x4000C000
#define LPC_TIMER16_1_BASE        0x40010000
#define LPC_TIMER32_0_BASE        0x40014000
#define LPC_TIMER32_1_BASE        0x40018000
#define LPC_ADC_BASE              0x4001C000
#define LPC_PMU_BASE              0x40038000
#define LPC_FLASH_BASE            0x4003C000
#define LPC_SSP0_BASE             0x40040000
#define LPC_SSP1_BASE             0x40058000
#define LPC_IOCON_BASE            0x40044000
#define LPC_SYSCTL_BASE           0x40048000
#if defined(CHIP_LPC1347)
#define LPC_GPIO_PIN_INT_BASE     0x4004C000
#define LPC_GPIO_GROUP_INT0_BASE  0x4005C000
#define LPC_GPIO_GROUP_INT1_BASE  0x40060000
#define LPC_GPIO_PORT_BASE        0x50000000
#define LPC_USB0_BASE             0x40080000
#define LPC_RITIMER_BASE          0x40064000
#else /*CHIP_LPC1343*/
#define LPC_GPIO_PORT0_BASE       0x50000000
#define LPC_GPIO_PORT1_BASE       0x50010000
#define LPC_GPIO_PORT2_BASE       0x50020000
#define LPC_GPIO_PORT3_BASE       0x50030000
#define LPC_USB0_BASE             0x40020000
#endif

/* Normalize types */
typedef IP_WWDT_001_T LPC_WWDT_T;
typedef IP_USART_001_T LPC_USART_T;
typedef IP_SSP_001_T LPC_SSP_T;
typedef IP_TIMER_001_T LPC_TIMER_T;
typedef IP_I2C_001_T LPC_I2C_T;
typedef IP_ADC_001_T LPC_ADC_T;
#if defined(CHIP_LPC1347)
typedef IP_GPIO_001_T LPC_GPIO_T;
typedef IP_GPIOPININT_001_T LPC_GPIOPININT_T;
typedef IP_RITIMER_001_T LPC_RITIMER_T;
#else
typedef IP_GPIO_003_T LPC_GPIO_T;
#endif

#define LPC_ADC                   ((IP_ADC_001_T              *) LPC_ADC_BASE)
#define LPC_I2C                   ((IP_I2C_001_T              *) LPC_I2C_BASE)
#define LPC_WWDT                  ((IP_WWDT_001_T             *) LPC_WWDT_BASE)
#define LPC_USART                 ((IP_USART_001_T            *) LPC_USART_BASE)
#define LPC_TIMER16_0             ((IP_TIMER_001_T            *) LPC_TIMER16_0_BASE)
#define LPC_TIMER16_1             ((IP_TIMER_001_T            *) LPC_TIMER16_1_BASE)
#define LPC_TIMER32_0             ((IP_TIMER_001_T            *) LPC_TIMER32_0_BASE)
#define LPC_TIMER32_1             ((IP_TIMER_001_T            *) LPC_TIMER32_1_BASE)
#define LPC_PMU                   ((LPC_PMU_T                 *) LPC_PMU_BASE)
#define LPC_FMC                   ((LPC_FMC_T                 *) LPC_FLASH_BASE)
#define LPC_SSP0                  ((IP_SSP_001_T              *) LPC_SSP0_BASE)
#define LPC_SSP1                  ((IP_SSP_001_T              *) LPC_SSP1_BASE)
#define LPC_IOCON                 ((LPC_IOCON_T               *) LPC_IOCON_BASE)
#define LPC_SYSCTL                ((LPC_SYSCTL_T              *) LPC_SYSCTL_BASE)
#if defined(CHIP_LPC1347)
#define LPC_GPIO_PIN_INT          ((IP_GPIOPININT_001_T       *) LPC_GPIO_PIN_INT_BASE)
#define LPC_GPIO_GROUP_INT0       ((IP_GPIOGROUPINT_001_T     *) LPC_GPIO_GROUP_INT0_BASE)
#define LPC_GPIO_GROUP_INT1       ((IP_GPIOGROUPINT_001_T     *) LPC_GPIO_GROUP_INT1_BASE)
#define LPC_GPIO_PORT             ((IP_GPIO_001_T             *) LPC_GPIO_PORT_BASE)
#define LPC_RITIMER               ((IP_RITIMER_001_T          *) LPC_RITIMER_BASE)
#define LPC_USB                   ((LPC_USB_T                 *) LPC_USB0_BASE)
#else
#define LPC_GPIO_PORT             ((IP_GPIO_003_T             *) LPC_GPIO_PORT0_BASE)
#endif

/**
 * @}
 */

#include "ring_buffer.h"
#include "sysctl_13xx.h"
#include "clock_13xx.h"
#include "fmc_13xx.h"
#include "timer_13xx.h"
#include "wwdt_13xx.h"
#include "adc_13xx.h"
#if defined(CHIP_LPC1347)
#include "ritimer_13xx.h"
#include "usbd_1347.h"
#include "flash_13xx.h"
#endif
#include "iocon_13xx.h"
#include "gpio_13xx.h"
#include "uart_13xx.h"
#include "ssp_13xx.h"
#include "i2c_13xx.h"

// FIXME - missing PMU support

#ifdef __cplusplus
}
#endif

#endif /* __CHIP_H_ */
