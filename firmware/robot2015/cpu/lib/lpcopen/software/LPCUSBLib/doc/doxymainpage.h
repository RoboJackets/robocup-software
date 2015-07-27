/**
 * @file 	doxymainpage.h
 * @brief 	This file just used to dummy information for LPCUSBlib document generating.
 * @author 	NXP LPCUSBlib Team.
 * @date 	26 Oct 2012
 */

/* Main page ------------------------------------------------------------------ */
/**

 */


/* Main-group definitions ----------------------------------------------------- */

/**
 * @defgroup LPCUSBlib LPCUSBlib
 * @ingroup USB_LPCUSBLIB
 * @{
 * @brief LPCUSBLib - A USB Library for NXP's MCU series
 * \image html LPCUSBLib_thumb.png
 * <b>
 * <p>LPCUSBlib is a full featured, open-source USB library designed
 * to run on all USB capable LPC microcontrollers from NXP.</p>
 * </b>
 * <p>The library includes support for</p>
 * <ul>
 * <li>USB 2.0</li>
 * <li>Host and device modes</li>
 * <li>Low, full and high speed transfer rates</li>
 * <li>Control, bulk, interrupt, and isochronous transfer types</li>
 * </ul>
 * <p>&nbsp;</p>
 * <div style="text-align: center;"><a
 * href="http://www.lpcware.com/content/project/LPCUSBlib/get-it">Visit
 * http://www.lpcware.com/content/project/LPCUSBlib to get the
 * latest version of LPCUSBlib</a></div>
 * <p>&nbsp;</p>
 *
 * \b <b>NXP's License:</b>
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
 *
 */


/** @defgroup Group_USBD On Chip USB Rom Drivers
  * @{
	*/

/** @defgroup Group_USBD_Class On Chip USB Rom Class Drivers
  * @{
	*/


  /**
  * @}
  */


  /**
  * @}
  */


/** @defgroup Group_USB_Files USB files list for specific targets
  * @{
  *
  * This section lists all needed files for building this library on 
  * following targets: host, device and for specific NXP MCUs.<br>
  * 
  * However, user may search for USB_CAN_BE_HOST or USB_CAN_BE_DEVICE
  * to find yourself those needed files to build for host or device.<br>
	*/

	/** @defgroup Group_USB_Files_Host USB files list for host build
	  * <b>Library common configurations:</b><br>
	  *    - software/LPCUSBLib/Common/*.h, take reference at @ref Group_Common.
	  *    - software/LPCUSBLib/LPCUSBlibConfig.h, take reference at @ref USB_Config.
	  *
	  * <b>Class driver:</b> take reference at @ref Group_USBClassDrivers.<br>
	  *    - software/LPCUSBLib/Drivers/USB/Class/Common/*.h
	  *    - software/LPCUSBLib/Drivers/USB/Class/Host/*.c and *.h
	  *
	  * <b>USB core:</b><br>
	  *    - software/LPCUSBLib/Drivers/USB/Core/ConfigDescriptor.h
	  *    - software/LPCUSBLib/Drivers/USB/Core/ConfigDescriptor.c
	  *    - software/LPCUSBLib/Drivers/USB/Core/Host.h
	  *    - software/LPCUSBLib/Drivers/USB/Core/Host.c
	  *    - software/LPCUSBLib/Drivers/USB/Core/HostStandardReq.h
	  *    - software/LPCUSBLib/Drivers/USB/Core/HostStandardReq.c
	  *    - software/LPCUSBLib/Drivers/USB/Core/Pipe.h
	  *    - software/LPCUSBLib/Drivers/USB/Core/Pipe.c
	  *    - software/LPCUSBLib/Drivers/USB/Core/PipeStream.h
	  *    - software/LPCUSBLib/Drivers/USB/Core/PipeStream.c
	  *    - software/LPCUSBLib/Drivers/USB/Core/StdDescriptors.h
	  *    - software/LPCUSBLib/Drivers/USB/Core/StdRequestType.h
	  *    - software/LPCUSBLib/Drivers/USB/Core/USBController.h
	  *    - software/LPCUSBLib/Drivers/USB/Core/USBController.c
	  *    - software/LPCUSBLib/Drivers/USB/Core/USBMemory.h
	  *    - software/LPCUSBLib/Drivers/USB/Core/USBMemory.c
	  *    - software/LPCUSBLib/Drivers/USB/Core/USBMode.h
	  *    - software/LPCUSBLib/Drivers/USB/Core/USBTask.h
	  *    - software/LPCUSBLib/Drivers/USB/Core/USBTask.c
	  *
	  * <b>USB host controller:</b><br>
	  *    - software/LPCUSBLib/Drivers/USB/Core/HCD/HCD.h
	  *    - software/LPCUSBLib/Drivers/USB/Core/HCD/HCD.c
	  *    - Build for EHCI architecture chips, ex: LPC18xx, LPC43xx, ...
	  *       + software/LPCUSBLib/Drivers/USB/Core/HCD/EHCI/EHCI.h
	  *       + software/LPCUSBLib/Drivers/USB/Core/HCD/EHCI/EHCI.c
	  *    - Build for OHCI architecture chips, ex: LPC17xx, LPC40xx, ...
	  *       + software/LPCUSBLib/Drivers/USB/Core/HCD/OHCI/OHCI.h
	  *       + software/LPCUSBLib/Drivers/USB/Core/HCD/OHCI/OHCI.c
	  *
	  * <b>Hardware abstract layer:</b> registers access.<br>
	  *    - software/LPCUSBLib/Drivers/USB/Core/HAL/HAL.h
	  *    - Build for LPC18xx, LPC43xx,... same USB core
	  *       + software/LPCUSBLib/Drivers/USB/Core/HAL/LPC18XX/HAL_LPC18xx.h
	  *       + software/LPCUSBLib/Drivers/USB/Core/HAL/LPC18XX/HAL_LPC18xx.c
	  *    - Build for LPC17xx, LPC40xx,... same USB core
	  *       + software/LPCUSBLib/Drivers/USB/Core/HAL/LPC17XX/HAL_LPC17xx.h
	  *       + software/LPCUSBLib/Drivers/USB/Core/HAL/LPC17XX/HAL_LPC17xx.c
	  *
	  * <b>USB register map:</b><br>
	  *    - Build for LPC175x_6x: software/lpc_core/lpc_chip/chip_17xx_40xx/chip_lpc175x_6x.h
	  *    - Build for LPC177x_8x: software/lpc_core/lpc_chip/chip_17xx_40xx/chip_lpc177x_8x.h
	  *    - Build for LPC407x_8x: software/lpc_core/lpc_chip/chip_17xx_40xx/chip_lpc407x_8x.h
	  *    - Build for LPC18xx: software/lpc_core/lpc_chip/chip_18xx_43xx/chip_lpc18xx.h
	  *    - Build for LPC43xx: software/lpc_core/lpc_chip/chip_18xx_43xx/chip_lpc43xx.h
	  * @{
		*/  
	  /**
	  * @}
	  */

	/** @defgroup Group_USB_Files_Device USB files list for device build
	  * <b>Library common configurations:</b><br>
	  *    - software/LPCUSBLib/Common/*.h, take reference at @ref Group_Common.
	  *    - software/LPCUSBLib/LPCUSBlibConfig.h, take reference at @ref USB_Config.
	  *
	  * <b>Class driver:</b> take reference at @ref Group_USBClassDrivers.<br>
	  *    - software/LPCUSBLib/Drivers/USB/Class/Common/*.h
	  *    - software/LPCUSBLib/Drivers/USB/Class/Device/*.c and *.h
	  *
	  * <b>USB core:</b><br>
	  *    - software/LPCUSBLib/Drivers/USB/Core/Device.h
	  *    - software/LPCUSBLib/Drivers/USB/Core/Device.c
	  *    - software/LPCUSBLib/Drivers/USB/Core/DeviceStandardReq.h
	  *    - software/LPCUSBLib/Drivers/USB/Core/DeviceStandardReq.c
	  *    - software/LPCUSBLib/Drivers/USB/Core/Endpoint.h
	  *    - software/LPCUSBLib/Drivers/USB/Core/Endpoint.c
	  *    - software/LPCUSBLib/Drivers/USB/Core/EndpointStream.h
	  *    - software/LPCUSBLib/Drivers/USB/Core/EndpointStream.c
	  *    - software/LPCUSBLib/Drivers/USB/Core/StdDescriptors.h
	  *    - software/LPCUSBLib/Drivers/USB/Core/StdRequestType.h
	  *    - software/LPCUSBLib/Drivers/USB/Core/USBController.h
	  *    - software/LPCUSBLib/Drivers/USB/Core/USBController.c
	  *    - software/LPCUSBLib/Drivers/USB/Core/USBMode.h
	  *    - software/LPCUSBLib/Drivers/USB/Core/USBTask.h
	  *    - software/LPCUSBLib/Drivers/USB/Core/USBTask.c
	  *
	  * <b>USB device controller:</b><br>
	  *    - software/LPCUSBLib/Drivers/USB/Core/DCD/EndpointCommon.h
	  *    - Build for LPC11Uxx, LPC1345_46_47,... same USB device core
	  *       + software/LPCUSBLib/Drivers/USB/Core/DCD/LPC11UXX/Device_LPC11Uxx.h
	  *       + software/LPCUSBLib/Drivers/USB/Core/DCD/LPC11UXX/Endpoint_LPC11Uxx.h
	  *       + software/LPCUSBLib/Drivers/USB/Core/DCD/LPC11UXX/Endpoint_LPC11Uxx.c
	  *    - Build for LPC17xx, LPC40xx, ... same USB device core
	  *       + software/LPCUSBLib/Drivers/USB/Core/DCD/LPC17XX/Device_LPC17xx.h
	  *       + software/LPCUSBLib/Drivers/USB/Core/DCD/LPC17XX/Endpoint_LPC17xx.h
	  *       + software/LPCUSBLib/Drivers/USB/Core/DCD/LPC17XX/Endpoint_LPC17xx.c
	  *    - Build for LPC18xx, LPC43xx, ... same USB device core
	  *       + software/LPCUSBLib/Drivers/USB/Core/DCD/LPC18XX/Device_LPC18xx.h
	  *       + software/LPCUSBLib/Drivers/USB/Core/DCD/LPC18XX/Endpoint_LPC18xx.h
	  *       + software/LPCUSBLib/Drivers/USB/Core/DCD/LPC18XX/Endpoint_LPC18xx.c
	  *
	  * <b>Hardware abstract layer:</b> registers access.<br>
	  *    - software/LPCUSBLib/Drivers/USB/Core/HAL/HAL.h
	  *    - Build for LPC18xx, LPC43xx,... same USB core
	  *       + software/LPCUSBLib/Drivers/USB/Core/HAL/LPC18XX/HAL_LPC18xx.h
	  *       + software/LPCUSBLib/Drivers/USB/Core/HAL/LPC18XX/HAL_LPC18xx.c
	  *    - Build for LPC17xx, LPC40xx,... same USB core
	  *       + software/LPCUSBLib/Drivers/USB/Core/HAL/LPC17XX/HAL_LPC17xx.h
	  *       + software/LPCUSBLib/Drivers/USB/Core/HAL/LPC17XX/HAL_LPC17xx.c
	  *    - Build for LPC11Uxx, LPC1345_46_47,... same USB core
	  *       + software/LPCUSBLib/Drivers/USB/Core/HAL/LPC11UXX/HAL_LPC11Uxx.h
	  *       + software/LPCUSBLib/Drivers/USB/Core/HAL/LPC11UXX/HAL_LPC11Uxx.c
	  *
	  * <b>USB register map:</b><br>
	  *    - Build for LPC11Uxx: software/lpc_core/lpc_chip/chip_11xx/chip.h
	  *    - Build for LPC1345_46_47: software/lpc_core/lpc_chip/chip_13xx/chip.h
	  *    - Build for LPC175x_6x: software/lpc_core/lpc_chip/chip_17xx_40xx/chip_lpc175x_6x.h
	  *    - Build for LPC177x_8x: software/lpc_core/lpc_chip/chip_17xx_40xx/chip_lpc177x_8x.h
	  *    - Build for LPC407x_8x: software/lpc_core/lpc_chip/chip_17xx_40xx/chip_lpc407x_8x.h
	  *    - Build for LPC18xx: software/lpc_core/lpc_chip/chip_18xx_43xx/chip_lpc18xx.h
	  *    - Build for LPC43xx: software/lpc_core/lpc_chip/chip_18xx_43xx/chip_lpc43xx.h
	  * @{
		*/  
	  /**
	  * @}
	  */

	/** @defgroup Group_USB_Files_Device_ROM USB files list for device ROM Stack build
	  * <b>Library common configurations:</b><br>
	  *    - software/LPCUSBLib/Common/*.h, take reference at @ref Group_Common.
	  *    - software/LPCUSBLib/LPCUSBlibConfig.h, take reference at @ref USB_Config.
	  *
	  * <b>Class and core driver:</b><br>
	  *    - software/LPCUSBLib/Drivers/USB/Core/DCD/USBRom/*.c and *.h, take reference at @ref Group_USBD
	  *    - software/LPCUSBLib/Drivers/USB/Core/USBController.h
	  *    - software/LPCUSBLib/Drivers/USB/Core/USBController.c
	  *    - software/LPCUSBLib/Drivers/USB/Core/USBMode.h
	  *    - software/LPCUSBLib/Drivers/USB/Core/USBTask.h
	  *    - software/LPCUSBLib/Drivers/USB/Core/USBTask.c
	  *
	  * <b>Hardware abstract layer:</b> registers access.<br>
	  *    - software/LPCUSBLib/Drivers/USB/Core/HAL/HAL.h
	  *    - Build for LPC18xx, LPC43xx,... same USB core
	  *       + software/LPCUSBLib/Drivers/USB/Core/HAL/LPC18XX/HAL_LPC18xx.h
	  *       + software/LPCUSBLib/Drivers/USB/Core/HAL/LPC18XX/HAL_LPC18xx.c
	  *    - Build for LPC11U2x, LPC1345_46_47,... same USB core
	  *       + software/LPCUSBLib/Drivers/USB/Core/HAL/LPC11UXX/HAL_LPC11Uxx.h
	  *       + software/LPCUSBLib/Drivers/USB/Core/HAL/LPC11UXX/HAL_LPC11Uxx.c
	  *
	  * <b>USB register map:</b><br>
	  *    - Build for LPC11U2x: software/lpc_core/lpc_chip/chip_11xx/chip.h
	  *    - Build for LPC1345_46_47: software/lpc_core/lpc_chip/chip_13xx/chip.h
	  *    - Build for LPC18xx: software/lpc_core/lpc_chip/chip_18xx_43xx/chip_lpc18xx.h
	  *    - Build for LPC43xx: software/lpc_core/lpc_chip/chip_18xx_43xx/chip_lpc43xx.h
	  * @{
		*/  
	  /**
	  * @}
	  */
	  
  /**
  * @}
  */

  
/**
 * @}
 */

/* End Of File ---------------------------------------------------------------- */

