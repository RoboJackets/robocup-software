/*
*********************************************************************************************************
*                                              uC/OS-II
*                                        The Real-Time Kernel
*
*                          (c) Copyright 2004-2009; Micrium, Inc.; Weston, FL               
*
*               All rights reserved.  Protected by international copyright laws.
*
*               uC/OS-II is provided in source form for FREE evaluation, for educational
*               use or peaceful research.  If you plan on using uC/OS-II in a commercial    
*               product you need to contact Micrium to properly license its use in your     
*               product.  We provide ALL the source code for your convenience and to        
*               help you experience uC/OS-II.  The fact that the source code is provided
*               does NOT mean that you can use it without paying a licensing fee.
*
*               Knowledge of the source code may NOT be used to develop a similar product.
*
*               Please help us continue to provide the Embedded community with the finest
*               software available.  Your honesty is greatly appreciated.
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*
*                                       APPLICATION CONFIGURATION
*
*                                      KEIL MCB1700 DEVELOPMENT KIT
*
* Filename      : app_cfg.h
* Version       : V1.00
* Programmer(s) : FT
*********************************************************************************************************
*/

#ifndef  APP_CFG_MODULE_PRESENT
#define  APP_CFG_MODULE_PRESENT


/*
*********************************************************************************************************
*                                       ADDITIONAL uC/MODULE ENABLES
*********************************************************************************************************
*/

#define  APP_CFG_PROBE_COM_EN                   DEF_DISABLED
#define  APP_CFG_SERIAL_EN                      DEF_DISABLED


/*
*********************************************************************************************************
*                                            TASK PRIORITIES
*********************************************************************************************************
*/

#define  APP_CFG_TASK_START_PRIO                           2u


/*
*********************************************************************************************************
*                                            TASK STACK SIZES
*********************************************************************************************************
*/

#define  APP_CFG_TASK_START_STK_SIZE                     256u


/*
*********************************************************************************************************
*                                          TASK STACK SIZES LIMIT
*********************************************************************************************************
*/

#define  APP_CFG_TASK_START_STK_SIZE_PCT_FULL             90u

#define  APP_CFG_TASK_START_STK_SIZE_LIMIT       (APP_CFG_TASK_START_STK_SIZE * (100u - APP_CFG_TASK_START_STK_SIZE_PCT_FULL))   / 100u


/*
*********************************************************************************************************
*                                          uC/LIB CONFIGURATION
*********************************************************************************************************
*/

#include <lib_cfg.h>


/*
*********************************************************************************************************
*                                          uC/SERIAL CONFIGURATION
*********************************************************************************************************
*/

#include <serial_cfg.h>


/*
*********************************************************************************************************
*                                    uC/SERIAL APPLICATION CONFIGURATION
*********************************************************************************************************
*/

#define  APP_SERIAL_CFG_TRACE_EN                DEF_DISABLED
#define  APP_SERIAL_CFG_TRACE_PORT_NAME         "UART0"


/*
*********************************************************************************************************
*                                       uC/PROBE APPLICATION CONFIGURATION
*********************************************************************************************************
*/

#define  APP_CFG_PROBE_SERIAL_PORT_NAME         "UART3"


/*
*********************************************************************************************************
*                                       TRACE / DEBUG CONFIGURATION
*********************************************************************************************************
*/

#define  TRACE_LEVEL_OFF                                   0
#define  TRACE_LEVEL_INFO                                  1
#define  TRACE_LEVEL_DBG                                   2

#define  APP_CFG_TRACE_LEVEL                    TRACE_LEVEL_OFF
#define  APP_CFG_TRACE                          printf

#define  APP_TRACE_INFO(x)                     ((APP_CFG_TRACE_LEVEL >= TRACE_LEVEL_INFO)  ? (void)(APP_CFG_TRACE x) : (void)0)
#define  APP_TRACE_DBG(x)                      ((APP_CFG_TRACE_LEVEL >= TRACE_LEVEL_DBG)   ? (void)(APP_CFG_TRACE x) : (void)0)

#endif
