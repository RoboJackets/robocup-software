/*
*********************************************************************************************************
*                                     SERIAL (BYTE) COMMUNICATION
*
*                         (c) Copyright 2007-2009; Micrium, Inc.; Weston, FL
*
*               All rights reserved.  Protected by international copyright laws.
*               Knowledge of the source code may NOT be used to develop a similar product.
*               Please help us continue to provide the Embedded community with the finest
*               software available.  Your honesty is greatly appreciated.
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*
*                                     SERIAL (BYTE) COMMUNICATION
*
*                                      CONFIGURATION TEMPLATE FILE
*
* Filename      : serial_cfg.h
* Version       : V2.00
* Programmer(s) : FGK
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*                                               SERIAL
*
* Note(s) : (1) Configure SERIAL_CFG_MAX_NBR_IF to the number of interfaces (i.e., UARTs) that will be
*               present.
*
*           (2) Configure SERIAL_CFG_RD_BUF_EN to enable/disable read buffer functionality.  The serial
*               core stores received data in the read buffer until the user requests it, providing a
*               reliable guarantee against receive overrun.
*
*           (3) Configure SERIAL_CFG_WR_BUF_EN to enable/disable write buffer functionality.  The serial
*               core stores line driver transmit data in the write buffer while the serial interface is
*               transmitting application data.
*
*           (4) Configure SERIAL_CFG_ARG_CHK_EXT_EN to enable/disable extended argument checking
*               functionality.
*
*           (5) Configure SERIAL_CFG_TX_DESC_NBR to allow multiple transmit operations (i.e., Serial_Wr,
*               Serial_WrAsync) to be queued.
*********************************************************************************************************
*/

#define  SERIAL_CFG_MAX_NBR_IF                             2    /* See Note #1.                                         */

#define  SERIAL_CFG_RD_BUF_EN                    DEF_DISABLED   /* See Note #2.                                         */

#define  SERIAL_CFG_WR_BUF_EN                    DEF_DISABLED   /* See Note #3.                                         */

#define  SERIAL_CFG_ARG_CHK_EXT_EN               DEF_DISABLED   /* See Note #4.                                         */

#define  SERIAL_CFG_TX_DESC_NBR                            1    /* See Note #5.                                         */
