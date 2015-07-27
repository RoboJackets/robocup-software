/*
 * @brief Hitex EVA 1850/4350 board file
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

#ifndef __BOARD_HITEX_EVA_18504350_H_
#define __BOARD_HITEX_EVA_18504350_H_

#include "chip.h"
#include "board_api.h"
#include "lpc_phy.h"
#include "uda1380.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @defgroup BOARD_HITEX_EVA_18504350 LPC1850 and LPC4350 Hitex EVA board support functions
 * @ingroup BOARDS_18XX_43XX
 * @{
 */

/** @defgroup BOARD_HITEX_EVA_1850_OPTIONS BOARD: LPC1850 Hitex EVA board builds options
 * This board has options that configure its operation at build-time.<br>
 *
 * For more information on driver options see<br>
 * @ref LPCOPEN_DESIGN_ARPPROACH<br>
 * @{
 */

/**
 * @}
 */

/** @defgroup BOARD_HITEX_EVA_4350_OPTIONS BOARD: LPC4350 Hitex EVA board builds options
 * This board has options that configure its operation at build-time.<br>
 *
 * For more information on driver options see<br>
 * @ref LPCOPEN_DESIGN_ARPPROACH<br>
 * @{
 */

/**
 * @}
 */

/**
 * HITEX board defintion, can be used in examples for board specific code
 */
#define BOARD_HITEX_EVA_18504350

/* For USBLIB examples */
#define LEDS_LED1           0x01
#define LEDS_LED2           0x02
#define LEDS_LED3           0x04
#define LEDS_LED4           0x08
#define LEDS_NO_LEDS        0x00
#define BUTTONS_BUTTON1     0x01
#define JOY_UP              0x01
#define JOY_DOWN            0x02
#define JOY_LEFT            0x04
#define JOY_RIGHT           0x08
#define JOY_PRESS           0x10
#define NO_BUTTON_PRESSED   0x00

#define BUTTONS_BUTTON1_GPIO_PORT_NUM   6
#define BUTTONS_BUTTON1_GPIO_BIT_NUM    21

#define I2CDEV_PCA9502_ADDR     (0x9A >> 1)
#define PCA9502_REG_IODIR       0x0A
#define PCA9502_REG_IOSTATE     0x0B
#define PCA9502_REG_IOINTENA    0x0C
#define PCA9502_REG_IOCONTROL   0x0E
#define PCA9502_REG_ADDR(x)     (((x) & 0x0F) << 3)

/*Define if use SDCARD for Mass Storage Example*/
// #define CFG_SDCARD

/**
 * Address of I2C device (UDA1380 CODEC) on board
 */
#define I2CDEV_UDA1380_ADDR     (0x34 >> 1)

/**
 * I2C Peripheral to which UDA1380 is connected
 */
#define UDA1380_I2C_BUS          I2C0

/**
 * Default location of LCD buffer is in DRAM
 */
#define FRAMEBUFFER_ADDR        0x28000000

/**
 * LCD configuration data
 */
extern const LCD_Config_T EA320x240;

/**
 * Default LCD configuration data for examples
 */
#define BOARD_LCD EA320x240

/**
 * @brief	Initialize pin muxing for a UART
 * @param	pUART	: Pointer to UART register block for UART pins to init
 * @return	Nothing
 */
void Board_UART_Init(LPC_USART_T *pUART);

/**
 * @brief	Initialize button(s) interface on board
 * @return	Nothing
 */
void Board_Buttons_Init(void);

/**
 * @brief	Returns button(s) state on board
 * @return	Returns BUTTONS_BUTTON1 if button1 is pressed
 */
uint32_t Buttons_GetStatus(void);

/**
 * @brief	Initialize joystick interface on board
 * @return	Nothing
 */
void Board_Joystick_Init(void);

/**
 * @brief	Returns joystick states on board
 * @return	Returns a JOY_* value, ir JOY_PRESS or JOY_UP
 */
uint8_t Joystick_GetStatus(void);

/**
 * @brief	Returns the MAC address assigned to this board
 * @param	mcaddr : Pointer to 6-byte character array to populate with MAC address
 * @return	Nothing
 */
void Board_ENET_GetMacADDR(uint8_t *mcaddr);

/**
 * @brief	Sets up board specific ADC interface
 * @return	Nothing
 */
void Board_ADC_Init(void);

/**
 * @brief	Sets up board specific I2C interface
 * @param	id	: I2C Peripheral ID (I2C0 or I2C1)
 * @return	Nothing
 */
void Board_I2C_Init(I2C_ID_T id);

/**
 * @brief	Sets up I2C Fast Plus mode
 * @param	id	: Must always be I2C0
 * @return	Nothing
 * @note	This function must be called before calling
 *      	Chip_I2C_SetClockRate() to set clock rates above
 *      	normal range 100KHz to 400KHz. Only I2C0 supports
 *       	this mode.
 */
STATIC INLINE void Board_I2C_EnableFastPlus(I2C_ID_T id)
{
	Chip_SCU_I2C0PinConfig(I2C0_FAST_MODE_PLUS);
}

/**
 * @brief	Disable I2C Fast Plus mode and enables default mode
 * @param	id	: Must always be I2C0
 * @return	Nothing
 * @sa		Board_I2C_EnableFastPlus()
 */
STATIC INLINE void Board_I2C_DisableFastPlus(I2C_ID_T id)
{
	Chip_SCU_I2C0PinConfig(I2C0_STANDARD_FAST_MODE);
}

/**
 * @brief	Initialize the LCD interface
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
 * @return	Nothing
 */
void Board_SetLCDBacklight(uint8_t Intensity);

/**
 * @brief	Initialize pin muxing for SDMMC interface
 * @return	Nothing
 */
void Board_SDMMC_Init(void);

/**
 * @brief	Initialize pin muxing for SSP interface
 * @param	pSSP	: Pointer to SSP interface to initialize
 * @return	Nothing
 */
void Board_SSP_Init(LPC_SSP_T *pSSP);

/**
 * @brief	Initialize I2S interface for the board and UDA1380
 * @param	pI2S	: Pointer to I2S register interface used on this board
 * @param	micIn	: If 1 selects MIC as input device, If 0 selects LINE_IN
 * @return	Nothing
 */
void Board_Audio_Init(LPC_I2S_T *pI2S, int micIn);

/**
 * @brief	Initialize DAC interface for the board
 * @param	pDAC	: Pointer to DAC register interface used on this board
 * @return	Nothing
 */
void Board_DAC_Init(LPC_DAC_T *pDAC);

/**
 * @brief	FIXME
 * @param	Stream	: FIXME
 * @return	Nothing
 */
void Serial_CreateStream(void *Stream);

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* __BOARD_HITEX_EVA_18504350_H_ */
