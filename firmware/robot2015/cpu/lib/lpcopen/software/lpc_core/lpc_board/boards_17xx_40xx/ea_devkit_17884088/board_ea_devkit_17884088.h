/*
 * @brief Embedded Artists LPC1788 Development Kit board file
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

#ifndef __BOARD_EA_DEVKIT_17884088H_
#define __BOARD_EA_DEVKIT_17884088H_

#include "chip.h"
#include "board_api.h"
#include "lpc_phy.h"
#include "lpc_norflash.h"
#include "lpc_nandflash.h"
#include "lpc_nandflash_k9f1g.h"
#include "uda1380.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @defgroup BOARD_EA_DEVKIT_17884088 LPC1788 and LPC4088 Embedded Artists Development Kit support functions
 * @ingroup BOARDS_17XX_40XX
 * @{
 */

/** @defgroup BOARD_EA_DEVKIT_1788_OPTIONS BOARD: LPC1788 Embedded Artists Development Kit build options
 * This board has options that configure its operation at build-time.<br>
 *
 * For more information on driver options see<br>
 * @ref LPCOPEN_DESIGN_ARPPROACH<br>
 * @{
 */

/**
 * @}
 */

/** @defgroup BOARD_EA_DEVKIT_4088_OPTIONS BOARD: LPC4088 Embedded Artists Development Kit build options
 * This board has options that configure its operation at build-time.<br>
 *
 * For more information on driver options see<br>
 * @ref LPCOPEN_DESIGN_ARPPROACH<br>
 * @{
 */

/**
 * @}
 */

#define BOARD_EA_DEVKIT_17884088	/*!< Build for EA1788/4088 boards */

#define FRAMEBUFFER_ADDR        EMC_ADDRESS_DYCS0	/*!< Frame buffer address for LCD */

#define I2CDEV_UDA1380_ADDR     0x1A/*!< UDA1380 address */
#define UDA1380_I2C_BUS         I2C0/**< I2C Bus to which UDA1380 is connected */

/**
 * LED defines
 */
#define LEDS_LED1           0x01
#define LEDS_LED2           0x02
#define LEDS_LED3           0x04
#define LEDS_LED4           0x08
#define LEDS_NO_LEDS        0x00

/**
 * Button defines
 */
#define BUTTONS_BUTTON1     0x01
#define NO_BUTTON_PRESSED   0x00

/**
 * Joystick defines
 */
#define JOY_UP              0x01
#define JOY_DOWN            0x02
#define JOY_LEFT            0x04
#define JOY_RIGHT           0x08
#define JOY_PRESS           0x10

/**
 * LCD configuration data
 */
extern const LCD_Config_T EA320x240;

/**
 * Default LCD configuration data for examples
 */
#define BOARD_LCD EA320x240

/**
 * @brief	Returns the MAC address assigned to this board
 * @param	mcaddr : Pointer to 6-byte character array to populate with MAC address
 * @return	Nothing
 * @note    Returns the MAC address used by Ethernet
 */
void Board_ENET_GetMacADDR(uint8_t *mcaddr);

/**
 * @brief	Initialize pin muxing for a UART
 * @param	pUART	: Pointer to UART register block for UART pins to init
 * @return	Nothing
 */
void Board_UART_Init(LPC_USART_T *pUART);

/**
 * @brief	Sets up board specific I2S interface and UDA1380 CODEC
 * @param	pI2S	: I2S peripheral to use (Must be LPC_I2S)
 * @param	micIn	: If 1 MIC will be used as input device, if 0
 *          LINE_IN will be used as input to Audio Codec.
 * @return	Nothing
 */
void Board_Audio_Init(LPC_I2S_T *pI2S, int micIn);

/**
 * @brief	Initialize pin muxing for SSP interface
 * @param	pSSP	: Pointer to SSP interface to initialize
 * @return	Nothing
 */
void Board_SSP_Init(LPC_SSP_T *pSSP);

/**
 * @brief	Sets up board specific I2C interface
 * @param	id	: I2C peripheral ID (I2C0, I2C1 or I2C2)
 * @return	Nothing
 */
void Board_I2C_Init(I2C_ID_T id);

/**
 * @brief	Sets up I2C Fast Plus mode
 * @param	id	: Must always be I2C0
 * @return	Nothing
 * @note	This function must be called before calling
 *          Chip_I2C_SetClockRate() to set clock rates above
 *          normal range 100KHz to 400KHz. Only I2C0 supports
 *          this mode. All I2C slaves of I2C0
 *          are connected to P0.27 (SDA0) P0.28 (SCL0) none of
 *          them will be accessible in fast plus mode because in
 *          fast plus mode P5.2 will be SDA0 and P5.3 will be SCL0,
 *          make sure the Fast Mode Plus supporting slave devices
 *          are connected to these pins.
 */
STATIC INLINE void Board_I2C_EnableFastPlus(I2C_ID_T id)
{
	/* Free P0[27] & P0[28] as SDA0 and SCL0 respectively*/
	Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 27, IOCON_FUNC0);
	Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 28, IOCON_FUNC0);

	/* Use P5[2] & P5[3] as SDA0 and SCL0 respectively */
	Chip_IOCON_PinMuxSet(LPC_IOCON, 5, 2, IOCON_FUNC5 | IOCON_HIDRIVE_EN);
	Chip_IOCON_PinMuxSet(LPC_IOCON, 5, 3, IOCON_FUNC5 | IOCON_HIDRIVE_EN);
}

/**
 * @brief	Disable I2C Fast Plus mode and enables default mode
 * @param	id	: Must always be I2C0
 * @return	Nothing
 * @sa		Board_I2C_EnableFastPlus()
 */
STATIC INLINE void Board_I2C_DisableFastPlus(I2C_ID_T id)
{
	/* Free P5[2] & P5[3] as SDA0 and SCL0 respectively */
	Chip_IOCON_PinMuxSet(LPC_IOCON, 5, 2, IOCON_FUNC0);
	Chip_IOCON_PinMuxSet(LPC_IOCON, 5, 3, IOCON_FUNC0);

	/* Use P0[27] & P0[28] as SDA0 and SCL0 respectively*/
	Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 27, IOCON_FUNC1);
	Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 28, IOCON_FUNC1);
}

/**
 * @brief	Sets up board specific CAN interface
 * @param	pCAN	: Pointer to CAN interface to initialize
 * @return	Nothing
 */
void Board_CAN_Init(LPC_CAN_T *pCAN);

/**
 * @brief	Initialize LCD Interface
 * @return	Nothing
 */
void Board_LCD_Init(void);

/**
 * @brief	Initialize the LCD controller on the QVGA (320x240) TFT LCD
 * @return	Nothing
 */
void Board_InitLCDController(void);

/**
 * @brief	Initialize touchscreen controller
 * @return	Nothing
 */
void Board_InitTouchController(void);

/**
 * @brief	Get Touch coordinates
 * @param	pX	: Pointer to x-Coord to populate
 * @param	pY	: Pointer to y-Coord to populate
 * @return	true if touch is detected or false if otherwise
 */
bool Board_GetTouchPos(int16_t *pX, int16_t *pY);

/**
 * @brief	Set LCD Backlight
 * @param   Intensity   : Intensity value to be set in LCD
 * @return	Nothing
 */
void Board_SetLCDBacklight(uint8_t Intensity);

/**
 * @brief	Initialize buttons on the board
 * @return	Nothing
 */
void Board_Buttons_Init(void);

/**
 * @brief	Get button status
 * @return	status of button
 */
uint32_t Buttons_GetStatus(void);

/**
 * @brief	Initialize Joystick
 * @return	Nothing
 */
void Board_Joystick_Init(void);

/**
 * @brief	Get Joystick status
 * @return	status of Joystick
 */
uint8_t Joystick_GetStatus(void);

/**
 * @brief	Sets up board specific Event Monitor/Recorder interface
 * @return	Nothing
 */
void Board_RTC_EV_Init(void);

/**
 * @brief	Create Serial Stream
 * @param	Stream	: Pointer to stream
 * @return	Nothing
 */
void Serial_CreateStream(void *Stream);

/**
 * @brief	Sets up board specific SDC peripheral
 * @return	Nothing
 */
void Board_SDC_Init(void);

#if defined(CHIP_LPC407X_8X)
/**
 * @brief	Sets up board for CMP peripheral
 * @return	Nothing
 */
void Board_CMP_Init(void);

/**
 * @brief	Sets up board for SPIFI peripheral
 * @return	Nothing
 */
void Board_SPIFI_Init(void);

#endif

/**
 * @brief	Write on address lines and data lines which are being connected to NOR Flash
 * @param	addr	: value which will be placed on address lines
 * @param	data	: value which will be placed on data lines
 * @return	Nothing
 */
STATIC INLINE void Board_NorFlash_WriteCmd(uint32_t addr, uint16_t data)
{
	*((volatile uint16_t *) (EMC_ADDRESS_CS0 | (addr << 1))) = data;
}

/**
 * @brief	Read command data returned by NOR Flash
 * @param	addr	: value which will be placed on address lines
 * @return	Data returned by NOR Flash
 */
STATIC INLINE uint16_t Board_NorFlash_ReadCmdData(uint32_t addr)
{
	return *((volatile uint16_t *) (EMC_ADDRESS_CS0 | (addr << 1)));
}

/**
 * @brief	Write 16-bit data to NOR Flash
 * @param	addr	: Offset in NOR Flash
 * @param	data	: Data which will be written to NOR Flash
 * @return	Nothing
 */
STATIC INLINE void Board_NorFlash_WriteWord(uint32_t addr, uint16_t data)
{
	*((volatile uint16_t *) (EMC_ADDRESS_CS0 | addr)) = data;
}

/**
 * @brief	Read 16-bit data from NOR Flash
 * @param	addr	: Offset in NOR Flash
 * @return	Nothing
 */
STATIC INLINE uint16_t Board_NorFlash_ReadWord(uint32_t addr)
{
	return *((volatile uint16_t *) (EMC_ADDRESS_CS0 | addr));
}

#if defined(NAND_SUPPORTED_LOCKEDCS)
/**
 * @brief	Lock NAND CS state
 * @param	activeCS	: true to activate CS, false to deactivate
 * @return	Nothing
 */
STATIC INLINE void Board_NANDFLash_CSLatch(bool activeCS)
{
	/* NAND FLASH active GPIO/CS state is low */
	Chip_GPIO_WritePortBit(LPC_GPIO, 4, 31, !activeCS);
}

#endif

#define NANDFLASH_READY_PORT       2
#define NANDFLASH_READY_PIN        21

/**
 * @brief	Read 16-bit data from NOR Flash
 * @return	Nothing
 */
void Board_NANDFLash_Init(void);

/**
 * @brief	Poll NAND Ready/Busy signal
 * @return	true if ready, false if busy
 * @note	Polls the R/nB signal and returns the state
 */
STATIC INLINE bool Board_NANDFLash_GetReady(void)
{
	return (Chip_GPIO_ReadPortBit(LPC_GPIO, NANDFLASH_READY_PORT, NANDFLASH_READY_PIN)) ? true : false;
}

/**
 * @brief	Write a command to NAND Flash
 * @param	cmd	: Command byte
 * @return	Nothing
 */
STATIC INLINE void Board_NANDFLash_WriteCmd(uint8_t cmd)
{
	*((volatile uint8_t *) (EMC_ADDRESS_CS1 | (1 << 20))) = cmd;
}

/**
 * @brief	Write a address byte to NAND Flash
 * @param	addr	: Address byte
 * @return	Nothing
 */
STATIC INLINE void Board_NANDFLash_WriteAddr(uint8_t addr)
{
	*((volatile uint8_t *) (EMC_ADDRESS_CS1 | (1 << 19))) = addr;
}

/**
 * @brief	Write a byte to NAND Flash
 * @param	data	: Data byte
 * @return	Nothing
 */
STATIC INLINE void Board_NANDFLash_WriteByte(uint8_t data)
{
	*((volatile uint8_t *) (EMC_ADDRESS_CS1)) = data;
}

/**
 * @brief	Read data byte from Nand Flash
 * @return	Byte read from NAND FLASH
 */
STATIC INLINE uint8_t Board_NANDFLash_ReadByte(void)
{
	return *((volatile uint8_t *) (EMC_ADDRESS_CS1));
}

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* __BOARD_EA_DEVKIT_17884088H_ */
