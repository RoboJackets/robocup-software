/*
 * @brief LPC11xx basic chip inclusion file
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

#ifndef CORE_M0
#error CORE_M0 is not defined for the LPC11xx architecture
#error CORE_M0 should be defined as part of your compiler define list
#endif

#if !defined(CHIP_LPC110X) && !defined(CHIP_LPC11XXLV) && !defined(CHIP_LPC11AXX) && \
	!defined(CHIP_LPC11CXX) && !defined(CHIP_LPC11EXX) && !defined(CHIP_LPC11UXX)
#error CHIP_LPC110x/CHIP_LPC11XXLV/CHIP_LPC11AXX/CHIP_LPC11CXX/CHIP_LPC11EXX/CHIP_LPC11UXX is not defined!
#endif

#if !defined (ENABLE_UNTESTED_CODE)
#if defined(CHIP_LPC110X)
#error The LCP110X code has not been tested with a platform. This code should \
build without errors but may not work correctly for the device. To disable this \
error message, define ENABLE_UNTESTED_CODE.
#endif
#if defined(CHIP_LPC11XXLV)
#error The LPC11XXLV code has not been tested with a platform. This code should \
build without errors but may not work correctly for the device. To disable this \
error message, define ENABLE_UNTESTED_CODE.
#endif
#if defined(CHIP_LPC11AXX)
#error The LPC11AXX code has not been tested with a platform. This code should \
build without errors but may not work correctly for the device. To disable this \
error message, define ENABLE_UNTESTED_CODE.
#endif
#if defined(CHIP_LPC11EXX)
#error The LPC11EXX code has not been tested with a platform. This code should \
build without errors but may not work correctly for the device. To disable this \
error message, define ENABLE_UNTESTED_CODE.
#endif
#endif

/** @defgroup IP_LPC110x_FILES CHIP: LPC110x Chip layer required IP layer drivers
 * @ingroup CHIP_11XX_Drivers
 * This is a list of the IP drivers required for the LPC110x device family.<br>
 * (adc_001.c, adc_001.h) @ref IP_ADC_001<br>
 * (gpio_003.c, gpio_003.h) @ref IP_GPIO_003<br>
 * (ssp_001.c, ssp_001.h) @ref IP_SSP_001<br>
 * (timer_001.c, timer_001.h) @ref IP_TIMER_001<br>
 * (usart_004.c, usart_004.h) @ref IP_USART_004<br>
 * (wwdt_001.c, wwdt_001.h) @ref IP_WWDT_001<br>
 * @{
 */

/**
 * @}
 */

/** @defgroup IP_LPC11XXLV_FILES CHIP: LPC11XXLV Chip layer required IP layer drivers
 * @ingroup CHIP_11XX_Drivers
 * This is a list of the IP drivers required for the LPC11XXLV device family.<br>
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

/** @defgroup IP_LPC11CXX_FILES CHIP: LPC11CXX Chip layer required IP layer drivers
 * @ingroup CHIP_11XX_Drivers
 * This is a list of the IP drivers required for the LPC11CXX device family.<br>
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
 
/** @defgroup IP_LPC11UXX_FILES CHIP: LPC11UXX Chip layer required IP layer drivers
 * @ingroup CHIP_11XX_Drivers
 * This is a list of the IP drivers required for the LPC11UXX device family.<br>
 * (adc_001.c, adc_001.h) @ref IP_ADC_001<br>
 * (gpio_001.h) @ref IP_GPIO_001<br>
 * (gpiogrpint_001.c, gpiogrpint_001.h) @ref IP_GPIOGRPINT_001<br>
 * (gpiopinint_001.c, gpiopinint_001.h) @ref IP_GPIOPININT_001<br>
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

/** @defgroup IP_LPC11EXX_FILES CHIP: LPC11EXX Chip layer required IP layer drivers
 * @ingroup CHIP_11XX_Drivers
 * This is a list of the IP drivers required for the LPC11EXX device family.<br>
 * (adc_001.c, adc_001.h) @ref IP_ADC_001<br>
 * (gpio_001.h) @ref IP_GPIO_001<br>
 * (gpiogrpint_001.c, gpiogrpint_001.h) @ref IP_GPIOGRPINT_001<br>
 * (gpiopinint_001.c, gpiopinint_001.h) @ref IP_GPIOPININT_001<br>
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

/** @defgroup IP_LPC11AXX_FILES CHIP: LPC11AXX Chip layer required IP layer drivers
 * @ingroup CHIP_11XX_Drivers
 * This is a list of the IP drivers required for the LPC11AXX device family.<br>
 * (acmp_001.c, acmp_001.h) @ref IP_ACMP_001<br>
 * (adc_001.c, adc_001.h) @ref IP_ADC_001<br>
 * (dac_001.c, dac_001.h) @ref IP_DAC_001<br>
 * (gpio_001.h) @ref IP_GPIO_001<br>
 * (gpiogrpint_001.c, gpiogrpint_001.h) @ref IP_GPIOGRPINT_001<br>
 * (gpiopinint_001.c, gpiopinint_001.h) @ref IP_GPIOPININT_001<br>
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
 
/* Peripheral mapping per device
   Peripheral					Device(s)
   ----------------------------	------------------------------------------------------------------------------
   I2C(40000000)								CHIP_LPC11XXLV/	CHIP_LPC11CXX/	CHIP_LPC11UXX/	CHIP_LPC11EXX/	CHIP_LPC11AXX
   WDT(40004000)				CHIP_LPC110x/	CHIP_LPC11XXLV/	CHIP_LPC11CXX/	CHIP_LPC11UXX/	CHIP_LPC11EXX/	CHIP_LPC11AXX
   UART(40008000)				CHIP_LPC110x/	CHIP_LPC11XXLV/	CHIP_LPC11CXX
   USART/SMARTCARD(40008000)													CHIP_LPC11UXX/	CHIP_LPC11EXX/	CHIP_LPC11AXX
   TIMER0_16(4000C000)			CHIP_LPC110x/	CHIP_LPC11XXLV/	CHIP_LPC11CXX/	CHIP_LPC11UXX/	CHIP_LPC11EXX/	CHIP_LPC11AXX
   TIMER1_16(40010000)			CHIP_LPC110x/	CHIP_LPC11XXLV/	CHIP_LPC11CXX/	CHIP_LPC11UXX/	CHIP_LPC11EXX/	CHIP_LPC11AXX
   TIMER0_32(40014000)			CHIP_LPC110x/	CHIP_LPC11XXLV/	CHIP_LPC11CXX/	CHIP_LPC11UXX/	CHIP_LPC11EXX/	CHIP_LPC11AXX
   TIMER1_32(40018000)			CHIP_LPC110x/	CHIP_LPC11XXLV/	CHIP_LPC11CXX/	CHIP_LPC11UXX/	CHIP_LPC11EXX/	CHIP_LPC11AXX
   ADC(4001C000)				CHIP_LPC110x/	CHIP_LPC11XXLV/	CHIP_LPC11CXX/	CHIP_LPC11UXX/	CHIP_LPC11EXX/	CHIP_LPC11AXX
   DAC(40024000)																								CHIP_LPC11AXX
   ACMP(40028000)																								CHIP_LPC11AXX
   PMU(40038000)				CHIP_LPC110x/					CHIP_LPC11CXX/	CHIP_LPC11UXX/	CHIP_LPC11EXX
   FLASH_CTRL(4003C000)			CHIP_LPC110x/	CHIP_LPC11XXLV/	CHIP_LPC11CXX/	CHIP_LPC11UXX
   FLASH_EEPROM(4003C000)																		CHIP_LPC11EXX/	CHIP_LPC11AXX
   SPI0(40040000)				CHIP_LPC110x/	CHIP_LPC11XXLV/	CHIP_LPC11CXX
   SSP0(40040000)																CHIP_LPC11UXX/	CHIP_LPC11EXX/	CHIP_LPC11AXX
   IOCONF(40044000)				CHIP_LPC110x/	CHIP_LPC11XXLV/	CHIP_LPC11CXX/	CHIP_LPC11UXX/	CHIP_LPC11EXX/	CHIP_LPC11AXX
   SYSCON(40048000)				CHIP_LPC110x/	CHIP_LPC11XXLV/	CHIP_LPC11CXX/	CHIP_LPC11UXX/	CHIP_LPC11EXX/	CHIP_LPC11AXX
   GPIOINTS(4004C000)															CHIP_LPC11UXX/	CHIP_LPC11EXX/	CHIP_LPC11AXX
   USB(40080000)																CHIP_LPC11UXX
   CCAN(40050000)												CHIP_LPC11CXX
   SPI1(40058000)												CHIP_LPC11CXX
   SSP1(40058000)																CHIP_LPC11UXX/	CHIP_LPC11EXX/	CHIP_LPC11AXX
   GPIO_GRP0_INT(4005C000)														CHIP_LPC11UXX/	CHIP_LPC11EXX/	CHIP_LPC11AXX
   GPIO_GRP1_INT(40060000)														CHIP_LPC11UXX/	CHIP_LPC11EXX/	CHIP_LPC11AXX
   GPIO_PORT(50000000)															CHIP_LPC11UXX/	CHIP_LPC11EXX/	CHIP_LPC11AXX
   GPIO_PIO0(50000000)			CHIP_LPC110x/	CHIP_LPC11XXLV/	CHIP_LPC11CXX
   GPIO_PIO1(50010000)			CHIP_LPC110x/	CHIP_LPC11XXLV/	CHIP_LPC11CXX
   GPIO_PIO2(50020000)							CHIP_LPC11XXLV/	CHIP_LPC11CXX
   GPIO_PIO3(50030000)							CHIP_LPC11XXLV/	CHIP_LPC11CXX
 */
#if defined(CHIP_LPC11AXX)
#include "acmp_001.h"
#include "dac_001.h"
#endif

#include "adc_001.h"
#include "ssp_001.h"

#if defined(CHIP_LPC11UXX) || defined(CHIP_LPC11EXX) || defined(CHIP_LPC11AXX)
#include "gpio_001.h"
#include "gpiogrpint_001.h"
#include "gpiopinint_001.h"
#else
#include "gpio_003.h"
#endif
#if !defined(CHIP_LPC110x)
#include "i2c_001.h"
#endif

#include "timer_001.h"
#include "usart_004.h"
#include "wwdt_001.h"

/** @defgroup PERIPH_11XX_BASE CHIP: LPC11xx Peripheral addresses and register set declarations
 * @ingroup CHIP_11XX_Drivers
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
#define LPC_DAC_BASE              0x40024000
#define LPC_ACMP_BASE             0x40028000
#define LPC_PMU_BASE              0x40038000
#define LPC_FLASH_BASE            0x4003C000
#define LPC_SSP0_BASE             0x40040000
#define LPC_IOCON_BASE            0x40044000
#define LPC_SYSCTL_BASE           0x40048000
#define LPC_GPIO_PIN_INT_BASE     0x4004C000
#define LPC_USB0_BASE             0x40080000
#define LPC_CAN0_BASE             0x40050000
#define LPC_SSP1_BASE             0x40058000
#if defined(CHIP_LPC11UXX) || defined(CHIP_LPC11EXX) || defined(CHIP_LPC11AXX)
#define LPC_GPIO_GROUP_INT0_BASE  0x4005C000
#define LPC_GPIO_GROUP_INT1_BASE  0x40060000
#define LPC_GPIO_PORT_BASE        0x50000000
#else
#define LPC_GPIO_PORT0_BASE       0x50000000
#define LPC_GPIO_PORT1_BASE       0x50010000
#endif
#if defined(CHIP_LPC11XXLV) || defined(CHIP_LPC11CXX)
#define LPC_GPIO_PORT2_BASE       0x50020000
#define LPC_GPIO_PORT3_BASE       0x50030000
#endif

/* Normalize types */
typedef IP_WWDT_001_T LPC_WWDT_T;
typedef IP_USART_001_T LPC_USART_T;
typedef IP_SSP_001_T LPC_SSP_T;
typedef IP_TIMER_001_T LPC_TIMER_T;
typedef IP_ADC_001_T LPC_ADC_T;

#if !defined(CHIP_LPC110x)
typedef IP_I2C_001_T LPC_I2C_T;
#endif

#if defined(CHIP_LPC11UXX) || defined(CHIP_LPC11EXX) || defined(CHIP_LPC11AXX)
typedef IP_GPIO_001_T LPC_GPIO_T;
typedef IP_GPIOPININT_001_T LPC_GPIOPININT_T;
#else
typedef IP_GPIO_003_T LPC_GPIO_T;
#endif

#if defined(CHIP_LPC11AXX)
typedef ACMP_001_T LPC_CMP_T;
typedef IP_DAC_001_T LPC_DAC_T;
#endif

#if !defined(CHIP_LPC110x)
#define LPC_I2C                   ((IP_I2C_001_T              *) LPC_I2C_BASE)
#endif

#define LPC_WWDT                  ((IP_WWDT_001_T             *) LPC_WWDT_BASE)
#define LPC_USART                 ((IP_USART_001_T            *) LPC_USART_BASE)
#define LPC_TIMER16_0             ((IP_TIMER_001_T            *) LPC_TIMER16_0_BASE)
#define LPC_TIMER16_1             ((IP_TIMER_001_T            *) LPC_TIMER16_1_BASE)
#define LPC_TIMER32_0             ((IP_TIMER_001_T            *) LPC_TIMER32_0_BASE)
#define LPC_TIMER32_1             ((IP_TIMER_001_T            *) LPC_TIMER32_1_BASE)
#define LPC_ADC                   ((IP_ADC_001_T              *) LPC_ADC_BASE)

#if defined(CHIP_LPC11AXX)
#define LPC_DAC                   ((IP_DAC_001_T              *) LPC_DAC_BASE)
#define LPC_CMP                   ((LPC_CMP_T                 *) LPC_ACMP_BASE)
#endif

#define LPC_PMU                   ((LPC_PMU_T                 *) LPC_PMU_BASE)
#define LPC_FMC                   ((LPC_FMC_T                 *) LPC_FLASH_BASE)
#define LPC_SSP0                  ((IP_SSP_001_T              *) LPC_SSP0_BASE)
#define LPC_IOCON                 ((LPC_IOCON_T               *) LPC_IOCON_BASE)
#define LPC_SYSCTL                ((LPC_SYSCTL_T              *) LPC_SYSCTL_BASE)
#define LPC_GPIO_PIN_INT          ((IP_GPIOPININT_001_T       *) LPC_GPIO_PIN_INT_BASE)
#if defined(CHIP_LPC11CXX) || defined(CHIP_LPC11UXX) || defined(CHIP_LPC11EXX) || defined(CHIP_LPC11AXX)
#define LPC_SSP1                  ((IP_SSP_001_T              *) LPC_SSP1_BASE)
#endif
#if defined(CHIP_LPC11UXX) || defined(CHIP_LPC11EXX) || defined(CHIP_LPC11AXX)
#define LPC_GPIO_GROUP_INT0       ((IP_GPIOGROUPINT_001_T     *) LPC_GPIO_GROUP_INT0_BASE)
#define LPC_GPIO_GROUP_INT1       ((IP_GPIOGROUPINT_001_T     *) LPC_GPIO_GROUP_INT1_BASE)
#define LPC_GPIO_PORT             ((IP_GPIO_001_T             *) LPC_GPIO_PORT_BASE)
#else
#define LPC_GPIO_PORT             ((IP_GPIO_003_T             *) LPC_GPIO_PORT0_BASE)
#endif
#define LPC_USB                   ((LPC_USB_T                 *) LPC_USB0_BASE)

/**
 * @}
 */

#include "ring_buffer.h"
#include "adc_11xx.h"
#include "fmc_11xx.h"
#include "sysctl_11xx.h"
#include "clock_11xx.h"
#include "iocon_11xx.h"
#include "pmu_11xx.h"
#include "gpio_11xx.h"
#include "timer_11xx.h"
#include "uart_11xx.h"
#include "wwdt_11xx.h"
#include "acmp_11xx.h"
#include "ssp_11xx.h"
#include "i2c_11xx.h"
#include "usbd_11xx.h"
#if defined(CHIP_LPC11CXX)
#include "ccand_11xx.h"
#endif

#ifdef __cplusplus
}
#endif

#endif /* __CHIP_H_ */
