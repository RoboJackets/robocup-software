/*----------------------------------------------------------------------------
 * U S B  -  K e r n e l
 *----------------------------------------------------------------------------
 * Name:    mw_usbd_api.h
 * Purpose: USB Custom Configuration
 * Version: V1.20
 *----------------------------------------------------------------------------
 * This software is supplied "AS IS" without any warranties, express,
 * implied or statutory, including but not limited to the implied
 * warranties of fitness for purpose, satisfactory quality and
 * noninfringement. Keil extends you a royalty-free right to reproduce
 * and distribute executable files created using this software for use
 * on NXP ARM microcontroller devices only. Nothing else gives
 * you the right to use this software.
 *
 * Copyright (c) 2005-2009 Keil Software.
 * Adaption to LPCxxxx, Copyright (c) 2009 NXP.
 *----------------------------------------------------------------------------*/

#include "error.h"
#include "mw_usbd.h"

#ifndef __MW_USBD_API_H__
#define __MW_USBD_API_H__

typedef void* USBD_HANDLE_T;
typedef void* USBD_HID_HANDLE_T;

/* function pointer types */
typedef ErrorCode_t (*USB_CB_T) (USBD_HANDLE_T hUsb);
typedef ErrorCode_t (*USB_PARAM_CB_T) (USBD_HANDLE_T hUsb, uint32_t event);
/* USBD setup request and endpoint event handler type */
typedef ErrorCode_t (*USB_EP_HANDLER_T)(USBD_HANDLE_T hUsb, void* data, uint32_t event);

/* USB descriptors data structure */
typedef struct _USB_CORE_DESCS_T
{
  /* all pointers in this structure should be 4 byte aligned */
  //ALIGNED(4)
  uint8_t *device_desc;
  uint8_t *string_desc;
  uint8_t *full_speed_desc;
  uint8_t *high_speed_desc;
  uint8_t *device_qualifier;
} USB_CORE_DESCS_T;


/* Override functions */
typedef struct __USBD_OVERRIDES_T
{
  USB_CB_T USB_EvtSetupHandler;
  USB_CB_T USB_EvtOutHandler;
  USB_CB_T USB_ReqGetStatus;
  USB_CB_T USB_ReqGetDescriptor;
  USB_CB_T USB_ReqGetConfiguration;
  USB_CB_T USB_ReqSetConfiguration;
  USB_CB_T USB_ReqGetInterface;
  USB_CB_T USB_ReqSetInterface;
  USB_PARAM_CB_T USB_ReqSetClrFeature;
  USB_PARAM_CB_T USB_ReqVendor;
} USBD_OVERRIDES_T;


typedef struct __USBD_API_INIT_PARAM_T
{
  uint32_t usb_reg_base;
  uint32_t mem_base;
  uint32_t mem_size;
  uint8_t max_num_ep;  /* max number of endpoints supported by the HW */

  /* USB Device Events Callback Functions */
  USB_CB_T USB_Reset_Event;
  USB_CB_T USB_Suspend_Event;
  USB_CB_T USB_Resume_Event;
  USB_CB_T USB_WakeUp_Event;
  USB_CB_T USB_SOF_Event;
  /*0 - Clear, 1 - Set*/
  USB_PARAM_CB_T USB_WakeUpCfg;
  USB_PARAM_CB_T USB_Power_Event;
  USB_PARAM_CB_T USB_Error_Event;

  /* USB Core Events Callback Functions */
  USB_CB_T USB_Configure_Event;
  USB_CB_T USB_Interface_Event;
  USB_CB_T USB_Feature_Event;

  /* cache and mmu translation functions */
  uint32_t (* virt_to_phys)(void* vaddr);
  void (* cache_flush)(uint32_t* start_adr, uint32_t* end_adr);

} USBD_API_INIT_PARAM_T;

//#include "mw_usbd/mw_usbd_hw.h" Batjo
#include "mw_usbd_hw.h"

/* midleware API */
extern ErrorCode_t mwUSB_RegisterClassHandler(USBD_HANDLE_T hUsb, USB_EP_HANDLER_T pfn, void* data);
extern ErrorCode_t mwUSB_RegisterEpHandler(USBD_HANDLE_T hUsb, uint32_t ep_index, USB_EP_HANDLER_T pfn, void* data);
extern void mwUSB_SetupStage (USBD_HANDLE_T hUsb); 
extern void mwUSB_DataInStage(USBD_HANDLE_T hUsb);
extern void mwUSB_DataOutStage(USBD_HANDLE_T hUsb); 
extern void mwUSB_StatusInStage(USBD_HANDLE_T hUsb); 
extern void mwUSB_StatusOutStage(USBD_HANDLE_T hUsb);
extern void mwUSB_StallEp0(USBD_HANDLE_T hUsb);


/************************************************************
* Mass Storage API structures and function prototypes 
************************************************************/
typedef struct __USBD_MSC_INIT_PARAM_T
{
  /* memory allocation params */
  uint32_t  mem_base;
  uint32_t  mem_size;
  /* mass storage paramas */
  uint8_t*  InquiryStr; /* Data pointed by the pointer should be of global scope. */
  uint32_t  BlockCount;
  uint32_t  BlockSize;
  uint32_t  MemorySize;
  /* Pointer to the interface descriptor within the descriptor
  * array passed to USBD_Init(). Assumes both HS and FS use same
  * BULK endpoints. Also the mwMSC_init() assumes that both BULK_IN
  * and BULK_OUT endpoints have same number. If different the EP_ISR
  * handlers should be installed manual by user code. 
  */
  uint8_t* intf_desc;
  /* user defined functions */
  void (*MSC_Write)( uint32_t offset, uint8_t** src, uint32_t length); 
  void (*MSC_Read)( uint32_t offset, uint8_t** dst, uint32_t length);
  ErrorCode_t (*MSC_Verify)( uint32_t offset, uint8_t dst[], uint32_t length);
  /* optional call back for MSC_Write optimization */
  void (*MSC_GetWriteBuf)( uint32_t offset, uint8_t** buff_adr, uint32_t length); 
  /* user overridable function */
  ErrorCode_t (*MSC_Ep0_Hdlr) (USBD_HANDLE_T hUsb, void* data, uint32_t event);

} USBD_MSC_INIT_PARAM_T;

extern uint32_t mwMSC_GetMemSize(USBD_MSC_INIT_PARAM_T* param);
extern ErrorCode_t mwMSC_init(USBD_HANDLE_T hUsb, USBD_MSC_INIT_PARAM_T* param);

/************************************************************
* Device Firmware Upgrade (DFU) API structures and function prototypes 
************************************************************/
typedef struct __USBD_DFU_INIT_PARAM_T
{
  /* memory allocation params */
  uint32_t  mem_base;
  uint32_t  mem_size;
  /* DFU paramas */
  uint16_t wTransferSize;

  /* Pointer to the interface descriptor within the descriptor
  * array passed to USBD_Init().  
  */
  uint8_t* intf_desc;
  /* user defined functions */
  /* return DFU_STATUS_ values defined in mw_usbd_dfu.h */
  uint8_t (*DFU_Write)( uint32_t block_num, uint8_t** src, uint32_t length);
  /* return 
  * DFU_STATUS_ : values defined in mw_usbd_dfu.h in case of errors
  * 0 : If end of memory reached
  * length : Amount of data copied to destination buffer
  */
  uint32_t (*DFU_Read)( uint32_t block_num, uint8_t** dst, uint32_t length);
  /* callback called after download is finished */
  void (*DFU_Done)(void);
  /* callback called after USB_REQ_DFU_DETACH is recived */
  void (*DFU_Detach)(USBD_HANDLE_T hUsb);

  /* user overridable function */
  ErrorCode_t (*DFU_Ep0_Hdlr) (USBD_HANDLE_T hUsb, void* data, uint32_t event);

} USBD_DFU_INIT_PARAM_T;

uint32_t mwDFU_GetMemSize(USBD_DFU_INIT_PARAM_T* param);
extern ErrorCode_t mwDFU_init(USBD_HANDLE_T hUsb, USBD_DFU_INIT_PARAM_T* param);

/************************************************************
* Human Interface Device (HID) API structures and function prototypes 
************************************************************/
typedef struct _HID_REPORT_T {
  uint16_t len;
  uint8_t idle_time;
  uint8_t __pad;
  uint8_t* desc;
} USB_HID_REPORT_T;

typedef struct __USBD_HID_INIT_PARAM_T
{
  /* memory allocation params */
  uint32_t  mem_base;
  uint32_t  mem_size;
  /* HID paramas */
  uint8_t max_reports;

  /* Pointer to the interface descriptor within the descriptor
  * array passed to USBD_Init().  
  */
  uint8_t* intf_desc;
  USB_HID_REPORT_T* report_data;

  /* user defined functions */
  /* required functions */
  ErrorCode_t (*HID_GetReport)( USBD_HID_HANDLE_T hHid, USB_SETUP_PACKET* pSetup, uint8_t** pBuffer, uint16_t* length); 
  ErrorCode_t (*HID_SetReport)( USBD_HID_HANDLE_T hHid, USB_SETUP_PACKET* pSetup, uint8_t** pBuffer, uint16_t length);
  /* optional functions */
  ErrorCode_t (*HID_GetPhysDesc)( USBD_HID_HANDLE_T hHid, USB_SETUP_PACKET* pSetup, uint8_t** pBuf, uint16_t* length);
  ErrorCode_t (*HID_SetIdle)( USBD_HID_HANDLE_T hHid, USB_SETUP_PACKET* pSetup, uint8_t idleTime); 
  ErrorCode_t (*HID_SetProtocol)( USBD_HID_HANDLE_T hHid, USB_SETUP_PACKET* pSetup, uint8_t protocol); 
  ErrorCode_t (*HID_EpIn_Hdlr) (USBD_HANDLE_T hUsb, void* data, uint32_t event);
  ErrorCode_t (*HID_EpOut_Hdlr) (USBD_HANDLE_T hUsb, void* data, uint32_t event);

  /* user overridable function */
  ErrorCode_t (*HID_GetReportDesc)(USBD_HID_HANDLE_T hHid, USB_SETUP_PACKET* pSetup, uint8_t** pBuf, uint16_t* length);
  ErrorCode_t (*HID_Ep0_Hdlr) (USBD_HANDLE_T hUsb, void* data, uint32_t event);

} USBD_HID_INIT_PARAM_T;

extern uint32_t mwHID_GetMemSize(USBD_HID_INIT_PARAM_T* param);
extern ErrorCode_t mwHID_init(USBD_HANDLE_T hUsb, USBD_HID_INIT_PARAM_T* param);

#endif  /* __MW_USBD_API_H__ */
