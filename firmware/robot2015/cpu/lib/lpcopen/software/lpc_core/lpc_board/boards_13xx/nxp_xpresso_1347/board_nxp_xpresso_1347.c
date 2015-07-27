/*
 * @brief Xpresso 1347 board file
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

#include "board.h"

/** @ingroup BOARD_NXP_XPRESSO_1347
 * @{
 */

/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/

#define BUTTONS_BUTTON1_GPIO_PORT_NUM           0
#define BUTTONS_BUTTON1_GPIO_BIT_NUM            16

#define JOYSTICK_UP_GPIO_PORT_NUM               1
#define JOYSTICK_UP_GPIO_BIT_NUM                22
#define JOYSTICK_DOWN_GPIO_PORT_NUM             1
#define JOYSTICK_DOWN_GPIO_BIT_NUM              20
#define JOYSTICK_LEFT_GPIO_PORT_NUM             1
#define JOYSTICK_LEFT_GPIO_BIT_NUM              23
#define JOYSTICK_RIGHT_GPIO_PORT_NUM            1
#define JOYSTICK_RIGHT_GPIO_BIT_NUM             21
#define JOYSTICK_PRESS_GPIO_PORT_NUM            1
#define JOYSTICK_PRESS_GPIO_BIT_NUM             19

/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/

/* System Clock Frequency (Core Clock) */
uint32_t SystemCoreClock;

/*****************************************************************************
 * Private functions
 ****************************************************************************/

/*****************************************************************************
 * Public functions
 ****************************************************************************/

/* Initialize pin muxing for UART interface */
void Board_UART_Init(void)
{
	/* Pin Muxing has already been done during SystemInit */
}

/* Sends a character on the UART */
void Board_UARTPutChar(char ch)
{
#if defined(DEBUG_ENABLE)
	/* Wait for space in FIFO */
	while ((IP_UART_ReadLineStatus(DEBUG_UART) & UART_LSR_THRE) == 0) {}
	Chip_UART_SendByte(DEBUG_UART, (uint8_t) ch);
#endif
}

/* Gets a character from the UART, returns EOF if no character is ready */
int Board_UARTGetChar(void)
{
#if defined(DEBUG_ENABLE)
	if (Chip_UART_ReadLineStatus(DEBUG_UART) & UART_LSR_RDR) {
		return (int) Chip_UART_ReadByte(DEBUG_UART);
	}
#endif
	return EOF;
}

/* Outputs a string on the debug UART */
void Board_UARTPutSTR(char *str)
{
#if defined(DEBUG_ENABLE)
	while (*str != '\0') {
		Board_UARTPutChar(*str++);
	}
#endif
}

/* Initialize debug output via UART for board */
void Board_Debug_Init(void)
{
#if defined(DEBUG_ENABLE)
	Board_UART_Init();

	/* Setup UART for 115.2K8N1 */
	Chip_UART_Init(LPC_USART);
	Chip_UART_SetBaud(LPC_USART, 115200);
	Chip_UART_ConfigData(LPC_USART, (UART_LCR_WLEN8 | UART_LCR_SBS_1BIT));
	Chip_UART_SetupFIFOS(LPC_USART, (UART_FCR_FIFO_EN | UART_FCR_TRG_LEV2));
	Chip_UART_TXEnable(LPC_USART);
#endif
}

/* Initializes board LED(s) */
static void Board_LED_Init(void)
{
	/* Set the PIO_7 as output */
	Chip_GPIO_WriteDirBit(LPC_GPIO_PORT, 0, 7, true);
}

/* Sets the state of a board LED to on or off */
void Board_LED_Set(uint8_t LEDNumber, bool On)
{
	if (LEDNumber == 0)
		Chip_GPIO_WritePortBit(LPC_GPIO_PORT, 0, 7, On);
}

/* Returns the current state of a board LED */
bool Board_LED_Test(uint8_t LEDNumber)
{
	return Chip_GPIO_ReadPortBit(LPC_GPIO_PORT, 0, 7);
}

/* Update system core clock rate, should be called if the system has
   a clock rate change */
void SystemCoreClockUpdate(void)
{
	/* CPU core speed */
	SystemCoreClock = Chip_Clock_GetSystemClockRate();
}

/* Set up and initialize all required blocks and functions related to the
   board hardware */
void Board_Init(void)
{
	/* Sets up DEBUG UART */
	DEBUGINIT();

	/* Updates SystemCoreClock global var with current clock speed */
	SystemCoreClockUpdate();

	/* Initialize GPIO */
	Chip_GPIO_Init(LPC_GPIO_PORT);

	/* Initialize LEDs */
	Board_LED_Init();
}

/* Initialize pin muxing for SSP interface */
void Board_SSP_Init(LPC_SSP_T *pSSP)
{
	if (pSSP == LPC_SSP0) {
	}
	else {
		Chip_IOCON_PinMuxSet(LPC_IOCON, 1, 23, (IOCON_FUNC2 | IOCON_MODE_PULLUP)); /* PIO1_23 connected to SSEL1 */
		Chip_IOCON_PinMuxSet(LPC_IOCON, 1, 20, (IOCON_FUNC2 | IOCON_MODE_PULLUP)); /* PIO1_20 connected to SCK1 */
		Chip_IOCON_PinMuxSet(LPC_IOCON, 1, 21, (IOCON_FUNC2 | IOCON_MODE_PULLUP)); /* PIO1_21 connected to MISO1 */
		Chip_IOCON_PinMuxSet(LPC_IOCON, 1, 22, (IOCON_FUNC2 | IOCON_MODE_PULLUP)); /* PIO1_22 connected to MOSI1 */
	}
}

/* Configure pin for ADC channel 0 */
void Board_ADC_Init(void)
{
	/* Muxing already setup as part of SystemInit for AD0 */
}

/* Initialize buttons on the board */
void Board_Buttons_Init(void)
{
	Chip_GPIO_WriteDirBit(LPC_GPIO_PORT, BUTTONS_BUTTON1_GPIO_PORT_NUM, BUTTONS_BUTTON1_GPIO_BIT_NUM, false);
}

/* Get button status */
uint32_t Buttons_GetStatus(void)
{
	uint8_t ret = NO_BUTTON_PRESSED;
	if (Chip_GPIO_ReadPortBit(LPC_GPIO_PORT, BUTTONS_BUTTON1_GPIO_PORT_NUM, BUTTONS_BUTTON1_GPIO_BIT_NUM) == 0x00) {
		ret |= BUTTONS_BUTTON1;
	}
	return ret;
}

/* Initialize Joystick */
void Board_Joystick_Init(void)
{
	Chip_IOCON_PinMuxSet(LPC_IOCON, JOYSTICK_UP_GPIO_PORT_NUM, JOYSTICK_UP_GPIO_BIT_NUM, (IOCON_FUNC0 | IOCON_MODE_PULLUP));
	Chip_IOCON_PinMuxSet(LPC_IOCON, JOYSTICK_DOWN_GPIO_PORT_NUM, JOYSTICK_DOWN_GPIO_BIT_NUM, (IOCON_FUNC0 | IOCON_MODE_PULLUP));
	Chip_IOCON_PinMuxSet(LPC_IOCON, JOYSTICK_LEFT_GPIO_PORT_NUM, JOYSTICK_LEFT_GPIO_BIT_NUM, (IOCON_FUNC0 | IOCON_MODE_PULLUP));
	Chip_IOCON_PinMuxSet(LPC_IOCON, JOYSTICK_RIGHT_GPIO_PORT_NUM, JOYSTICK_RIGHT_GPIO_BIT_NUM, (IOCON_FUNC0 | IOCON_MODE_PULLUP));
	Chip_IOCON_PinMuxSet(LPC_IOCON, JOYSTICK_PRESS_GPIO_PORT_NUM, JOYSTICK_PRESS_GPIO_BIT_NUM, (IOCON_FUNC0 | IOCON_MODE_PULLUP));
	Chip_GPIO_WriteDirBit(LPC_GPIO_PORT, JOYSTICK_UP_GPIO_PORT_NUM, JOYSTICK_UP_GPIO_BIT_NUM, false);
	Chip_GPIO_WriteDirBit(LPC_GPIO_PORT, JOYSTICK_DOWN_GPIO_PORT_NUM, JOYSTICK_DOWN_GPIO_BIT_NUM, false);
	Chip_GPIO_WriteDirBit(LPC_GPIO_PORT, JOYSTICK_LEFT_GPIO_PORT_NUM, JOYSTICK_LEFT_GPIO_BIT_NUM, false);
	Chip_GPIO_WriteDirBit(LPC_GPIO_PORT, JOYSTICK_RIGHT_GPIO_PORT_NUM, JOYSTICK_RIGHT_GPIO_BIT_NUM, false);
	Chip_GPIO_WriteDirBit(LPC_GPIO_PORT, JOYSTICK_PRESS_GPIO_PORT_NUM, JOYSTICK_PRESS_GPIO_BIT_NUM, false);
}

/* Get Joystick status */
uint8_t Joystick_GetStatus(void)
{
	uint8_t ret = NO_BUTTON_PRESSED;
	if ((Chip_GPIO_ReadPortBit(LPC_GPIO_PORT, JOYSTICK_UP_GPIO_PORT_NUM, JOYSTICK_UP_GPIO_BIT_NUM)) == 0x00) {
		ret |= JOY_UP;
	}
	else if (Chip_GPIO_ReadPortBit(LPC_GPIO_PORT, JOYSTICK_DOWN_GPIO_PORT_NUM, JOYSTICK_DOWN_GPIO_BIT_NUM) == 0x00) {
		ret |= JOY_DOWN;
	}
	else if ((Chip_GPIO_ReadPortBit(LPC_GPIO_PORT, JOYSTICK_LEFT_GPIO_PORT_NUM, JOYSTICK_LEFT_GPIO_BIT_NUM)) == 0x00) {
		ret |= JOY_LEFT;
	}
	else if (Chip_GPIO_ReadPortBit(LPC_GPIO_PORT, JOYSTICK_RIGHT_GPIO_PORT_NUM, JOYSTICK_RIGHT_GPIO_BIT_NUM) == 0x00) {
		ret |= JOY_RIGHT;
	}
	else if ((Chip_GPIO_ReadPortBit(LPC_GPIO_PORT, JOYSTICK_PRESS_GPIO_PORT_NUM, JOYSTICK_PRESS_GPIO_BIT_NUM)) == 0x00) {
		ret |= JOY_PRESS;
	}
	return ret;
}

/**
 * @}
 */
