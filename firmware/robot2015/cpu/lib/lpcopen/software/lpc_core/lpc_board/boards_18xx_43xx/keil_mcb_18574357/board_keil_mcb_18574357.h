/*
 * @brief Keil MCB 1857/4357 board file
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

#ifndef __BOARD_KEIL_MCB_18574357_H_
#define __BOARD_KEIL_MCB_18574357_H_

#include "chip.h"
#include "board_api.h"
#include "lpc_phy.h"
#include "uda1380.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @defgroup BOARD_KEIL_MCB_18574357 LPC1857 and LPC4357 Keil MCB board support functions
 * @ingroup BOARDS_18XX_43XX
 * @{
 */

/** @defgroup BOARD_KEIL_MCB_1857_OPTIONS BOARD: LPC1857 Keil MCB board builds options
 * This board has options that configure its operation at build-time.<br>
 *
 * For more information on driver options see<br>
 * @ref LPCOPEN_DESIGN_ARPPROACH<br>
 * @{
 */

/**
 * @}
 */

/** @defgroup BOARD_KEIL_MCB_4357_OPTIONS BOARD: LPC4357 Keil MCB board builds options
 * This board has options that configure its operation at build-time.<br>
 *
 * For more information on driver options see<br>
 * @ref LPCOPEN_DESIGN_ARPPROACH<br>
 * @{
 */

/**
 * @}
 */

#define BOARD_KEIL_MCB_18574357

#define LED_NUMBER_OF 1

#define BUTTONS_BUTTON1_GPIO_PORT_NUM			2
#define BUTTONS_BUTTON1_GPIO_BIT_NUM			0
#define JOYSTICK_UP_GPIO_PORT_NUM               6
#define JOYSTICK_UP_GPIO_BIT_NUM                10
#define JOYSTICK_DOWN_GPIO_PORT_NUM             6
#define JOYSTICK_DOWN_GPIO_BIT_NUM              11
#define JOYSTICK_LEFT_GPIO_PORT_NUM             6
#define JOYSTICK_LEFT_GPIO_BIT_NUM              12
#define JOYSTICK_RIGHT_GPIO_PORT_NUM            6
#define JOYSTICK_RIGHT_GPIO_BIT_NUM             13
#define JOYSTICK_PRESS_GPIO_PORT_NUM            6
#define JOYSTICK_PRESS_GPIO_BIT_NUM             8

#define JOY_UP              0x01
#define JOY_DOWN            0x02
#define JOY_LEFT            0x04
#define JOY_RIGHT           0x08
#define JOY_PRESS           0x10
#define NO_BUTTON_PRESSED   0x00

#define BUTTONS_BUTTON1     0x01

#define LEDS_LED1           0x01
#define LEDS_LED2           0x02
#define LEDS_LED3           0x04
#define LEDS_LED4           0x08
#define LEDS_NO_LEDS        0x00

/* UDA1380 address */
#define I2CDEV_UDA1380_ADDR     (0x34 >> 1)
#define UDA1380_I2C_BUS         I2C0

/* Frame buffer address for lcd */
#define FRAMEBUFFER_ADDR        0x28000000

extern const LCD_Config_T MCB4300_LCD;
#define BOARD_LCD MCB4300_LCD

/** LCD controller definitions */
#define SSP_ID LPC_SSP0
#define C_GLCD_H_SIZE   240
#define C_GLCD_V_SIZE   320

/** Private types/definitions for touch screen controller (STMPE811) */

#define TSC_I2C_ADDR    (0x82 >> 1)	/* Touchscreen 7-bit I2C address */
#define TSC_I2C_BUS     I2C0

/** STMPE811 Register addresses */
#define SYS_CTRL1       0x03
#define SYS_CTRL2       0x04
#define INT_CTRL        0x09
#define INT_EN          0x0A
#define INT_STA         0x0B
#define GPIO_ALT_FUNCT  0x17
#define ADC_CTRL1       0x20
#define ADC_CTRL2       0x21
#define TSC_CTRL        0x40
#define TSC_CFG         0x41
#define FIFO_TH         0x4A
#define FIFO_STA        0x4B
#define FIFO_SIZE       0x4C
#define DATA_X          0x4D
#define DATA_Y          0x4F
#define DATA_Z          0x51
#define TSC_FRACTION_Z  0x56
#define TSC_I_DRIVE     0x58
#define TSC_SHIELD      0x59
#define DATA_XYZ        0xD7

/**
 * @brief	Sets up board specific ADC interface
 * @return	Nothing
 */
void Board_ADC_Init(void);

/**
 * @brief	Sets up board specific I2C interface
 * @param	id	: I2C Interface ID (I2C0, I2C1 ... etc)
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
 * @brief	Initializes board specific GPIO Interrupt
 * @return	Nothing
 */
void Board_GPIO_Int_Init(void);

/**
 * @brief	Sets up board specific SDMMC interface
 * @return	Nothing
 */
void Board_SDMMC_Init(void);

/**
 * @brief	Sets up board specific SSP interface
 * @param	pSSP	: Pointer to SSP interface to initialize
 * @return	Nothing
 */
void Board_SSP_Init(LPC_SSP_T *pSSP);

/**
 * @brief	Returns the MAC address assigned to this board
 * @param	mcaddr	: Pointer to 6-byte character array to populate with MAC address
 * @return	Nothing
 */
void Board_ENET_GetMacADDR(uint8_t *mcaddr);

/**
 * @brief	Initialize pin muxing for a UART
 * @param	pUART	: Pointer to UART register block for UART pins to init
 * @return	Nothing
 */
void Board_UART_Init(LPC_USART_T *pUART);

/**
 * @brief	Initialize the LCD interface
 * @return	Nothing
 */
void Board_LCD_Init(void);

/**
 * @brief	Initializes the LCD Controller
 * @return	Nothing
 */
void Board_InitLCDController(void);

/**
 * @brief	Initialize touchscreen controller
 * @return	Nothing
 */
void Board_InitTouchController(void);

/**
 * @brief	Get touch screen position (Polled mode)
 * @param	pX	: pointer to X position
 * @param	pY	: pointer to Y position
 * @return	true if touch is detected or false if otherwise
 * @note	When using OS use Board_I2C_GetTouchPos(), instead
 * 			of this function as this function uses busy wait
 * 			to do I2C transfers.
 */
bool Board_GetTouchPos(int16_t *pX, int16_t *pY);

/**
 * @brief	Get touch screen position (Interrupt Based)
 * @param	pX	: pointer to X position
 * @param	pY	: pointer to Y position
 * @return	true if touch is detected or false if otherwise
 * @note	To use this function, I2C interrupt for
 * 			#TSC_I2C_BUS must be enabled, and the corresponding
 * 			handler must call Chip_I2C_MasterStateHandler()
 */
bool Board_I2C_GetTouchPos(int16_t *pX, int16_t *pY);

/**
 * @brief	Set LCD Backlight
 * @return	Nothing
 */
void Board_SetLCDBacklight(uint8_t Intensity);

/**
 * @brief	Initializes board specific buttons
 * @return	Nothing
 */
void Board_Buttons_Init (void);

/**
 * @brief	Initializes board specific joystick
 * @return	Nothing
 */
void Board_Joystick_Init (void);

/**
 * @brief	Initialize joystick interface on board
 * @return	joystick status: up, down, left or right
 */
uint8_t Joystick_GetStatus (void);

/**
 * @brief	Returns button(s) state on board
 * @return	Returns BUTTONS_BUTTON1 if button1 is pressed
 */
uint32_t Buttons_GetStatus(void);

/**
 * @brief	Sets up board specific I2S interface and UDA1380
 * @param	pI2S	: Pointer to I2S interface to initialize
 * @param	micIn	: If 1 selects MIC as input device, if 0 selects LINE_IN
 * @return	Nothing
 */
void Board_Audio_Init(LPC_I2S_T *pI2S, int micIn);

/**
 * @brief	Initialize DAC interface for the board
 * @param	pDAC	: Pointer to DAC register interface used on this board
 * @return	Nothing
 */
void Board_DAC_Init(LPC_DAC_T *pDAC);

//FIXME Should we remove this function?
void Serial_CreateStream(void *Stream);

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* __BOARD_KEIL_MCB_18574357_H_ */
