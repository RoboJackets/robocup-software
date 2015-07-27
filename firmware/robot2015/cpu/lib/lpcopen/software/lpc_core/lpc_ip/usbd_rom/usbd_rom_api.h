/***********************************************************************
* $Id: mw_usbd_rom_api.h.rca 1.4 Tue Nov  1 11:45:07 2011 nlv09221 Experimental $
*
* Project: USB device ROM Stack
*
* Description:
*     ROM API Module definitions.
*
***********************************************************************
*   Copyright(C) 2011, NXP Semiconductor
*   All rights reserved.
*
* Software that is described herein is for illustrative purposes only
* which provides customers with programming information regarding the
* products. This software is supplied "AS IS" without any warranties.
* NXP Semiconductors assumes no responsibility or liability for the
* use of the software, conveys no license or title under any patent,
* copyright, or mask work right to the product. NXP Semiconductors
* reserves the right to make changes in the software without
* notification. NXP Semiconductors also make no representation or
* warranty that such application will be suitable for the specified
* use without further testing or modification.
**********************************************************************/
#ifndef __MW_USBD_ROM_API_H
#define __MW_USBD_ROM_API_H
/** \file
 *  \brief ROM API for USB device stack.
 *
 *  Definition of functions exported by ROM based USB device stack.
 *
 */

#include "error.h"
#include "usbd.h"
#include "usbd_hw.h"
#include "usbd_core.h"
#include "usbd_mscuser.h"
#include "usbd_dfuuser.h"
#include "usbd_hiduser.h"
#include "usbd_cdcuser.h"

/** \brief Main USBD API functions structure.
 *  \ingroup Group_USBD
 *
 *  This structure contains pointer to various USB Device stack's sub-module 
 *  function tables. This structure is used as main entry point to access
 *  various methods (grouped in sub-modules) exposed by ROM based USB device 
 *  stack.
 *
 */
typedef struct USBD_API 
{
  const USBD_HW_API_T* hw; /**< Pointer to function table which exposes functions 
                           which interact directly with USB device stack's core 
                           layer.*/
  const USBD_CORE_API_T* core; /**< Pointer to function table which exposes functions 
                           which interact directly with USB device controller 
                           hardware.*/
  const USBD_MSC_API_T* msc; /**< Pointer to function table which exposes functions 
                           provided by MSC function driver module.
                           */
  const USBD_DFU_API_T* dfu; /**< Pointer to function table which exposes functions 
                           provided by DFU function driver module.
                           */
  const USBD_HID_API_T* hid; /**< Pointer to function table which exposes functions 
                           provided by HID function driver module.
                           */
  const USBD_CDC_API_T* cdc; /**< Pointer to function table which exposes functions 
                           provided by CDC-ACM function driver module.
                           */
  const uint32_t* reserved6; /**< Reserved for future function driver module.
                           */
  const uint32_t version; /**< Version identifier of USB ROM stack. The version is
                          defined as 0x0CHDMhCC where each nibble represnts version 
                          number of the corresponding component.
                          CC -  7:0  - 8bit core version number
                           h - 11:8  - 4bit hardware interface version number
                           M - 15:12 - 4bit MSC class module version number
                           D - 19:16 - 4bit DFU class module version number
                           H - 23:20 - 4bit HID class module version number
                           C - 27:24 - 4bit CDC class module version number
                           H - 31:28 - 4bit reserved 
                           */

} USBD_API_T;

/* A table of pointers to the chip's main ROM functions contained in ROM is located at the
   address contained at this location */
typedef struct _ROM {
	const unsigned p_otp;
	const unsigned p_aes;
	const unsigned p_pwd;
	const unsigned p_clk;
	const unsigned p_ipc;
	const unsigned p_spifi;
	const unsigned p_usbd;
}  ROM_FUNCTION_TABLE;

/* A table of pointers to the USBD functions contained in ROM is located at the
   address contained at this location */
				#define ROM_FUNCTION_TABLE_PTR_ADDR         (0x10400104UL)
				#define ROM_USBD_PTR (((ROM_FUNCTION_TABLE *) (ROM_FUNCTION_TABLE_PTR_ADDR))->p_usbd)

#define USBD_API (((USBD_API_T*)(ROM_USBD_PTR)))
//extern const  USBD_API_T usb_api;


#endif /*__MW_USBD_ROM_API_H*/

