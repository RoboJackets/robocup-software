/*
 * @brief NXP XPresso 812 board file
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
#include "string.h"

/** @ingroup BOARD_NXP_XPRESSO_812
 * @{
 */

/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/

#define RED_LED_PORT_BIT                7
#define BLUE_LED_PORT_BIT               16
#define GREEN_LED_PORT_BIT              17

static const uint8_t LED_BITS[] = {RED_LED_PORT_BIT, BLUE_LED_PORT_BIT, GREEN_LED_PORT_BIT};

/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/

/*****************************************************************************
 * Private functions
 ****************************************************************************/

/* Initialize the LEDs on the NXP LPC812 XPresso Board */
static void Board_LED_Init(void)
{
	uint8_t i;
	for (i = 0; i < sizeof(LED_BITS); i++) {
		Chip_GPIO_WriteDirBit(LPC_GPIO_PORT, LPC8XX_PORT_NUM, LED_BITS[i], true);
		Chip_GPIO_WritePortBit(LPC_GPIO_PORT, LPC8XX_PORT_NUM, LED_BITS[i], (bool) true);
	}
}

/*****************************************************************************
 * Public functions
 ****************************************************************************/

/* Set the LED to the state of "On" */
void Board_LED_Set(uint8_t LEDNumber, bool On)
{
	Chip_GPIO_WritePortBit(LPC_GPIO_PORT, LPC8XX_PORT_NUM, LED_BITS[LEDNumber], (bool) !On);
}

/* Return the state of LEDNumber */
bool Board_LED_Test(uint8_t LEDNumber)
{
	return (bool) !Chip_GPIO_ReadPortBit(LPC_GPIO_PORT, LPC8XX_PORT_NUM, LED_BITS[LEDNumber]);
}

/* System Clock Frequency (Core Clock) */
uint32_t SystemCoreClock;

/*
 * Update system core clock rate, should be called if the system has
 * a clock rate change
 */
void SystemCoreClockUpdate(void)
{
	/* CPU core speed */
	SystemCoreClock = Chip_Clock_GetSystemClockRate();
}

/* Board UART Initialisation function */
void Board_UART_Init(LPC_USART_T *pUART)
{
	/* Enable the clock to the Switch Matrix */
	Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_SWM);
	Chip_Clock_SetUARTClockDiv(1);	/* divided by 1 */
	if (pUART == LPC_USART0) {
		/*connect the U0_TXD_O and U0_RXD_I signals to port pins(P0.4, P0.0) */
		Chip_SWM_FixedPinEnable(ACMP_I1, DISABLE);
		Chip_SWM_MovablePinAssign(SWM_U0_TXD_O, PIO4);
		Chip_SWM_MovablePinAssign(SWM_U0_RXD_I, PIO0);
		/* Enable USART0 clock */
		Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_UART0);	// FIXME UART clocking and reset need to be part of CHIP driver
		/* Peripheral reset control to USART0, a "1" bring it out of reset. */
		Chip_SYSCTL_PeriphReset(RESET_USART0);
	}
	else if (pUART == LPC_USART1) {
		/*connect the U1_TXD_O and U1_RXD_I signals to port pins(P0.13, P0.14) */
		Chip_SWM_MovablePinAssign(SWM_U1_TXD_O, PIO13);
		Chip_SWM_MovablePinAssign(SWM_U1_RXD_I, PIO14);
		/* Enable USART1 clock */
		Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_UART1);	// FIXME UART clocking and reset need to be part of CHIP driver
		/* Peripheral reset control to USART1, a "1" bring it out of reset. */
		Chip_SYSCTL_PeriphReset(RESET_USART1);
	}
	else {
		/*connect the U2_TXD_O and U2_RXD_I signals to port pins(P0.13, P0.14) */
		Chip_SWM_MovablePinAssign(SWM_U2_TXD_O, PIO13);
		Chip_SWM_MovablePinAssign(SWM_U2_RXD_I, PIO14);
		/* Enable USART2 clock */
		Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_UART2);	// FIXME UART clocking and reset need to be part of CHIP driver
		/* Peripheral reset control to USART2, a "1" bring it out of reset. */
		Chip_SYSCTL_PeriphReset(RESET_USART2);
	}
}

/* Initialize pin muxing for SPI interface */
void Board_SPI_Init(LPC_SPI_T *pSPI)
{
	/* Enable the clock to the Switch Matrix */
	Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_SWM);
	if (pSPI == LPC_SPI0) {
		/*
		 * Initialize SSP0 pins connect
		 * SCK0: PINASSIGN3[31:24] : Select P0.12
		 * MOSI: PINASSIGN4[7:0] : Select P0.14
		 * MISO0: PINASSIGN4[15:8]: Select P0.6
		 * SSEL0: PINASSIGN4[23:16] : Select P0.13
		 */
		Chip_SWM_FixedPinEnable(VDDCMP, DISABLE);
		Chip_SWM_MovablePinAssign(SWM_SPI0_SCK_IO, PIO12);
		Chip_SWM_MovablePinAssign(SWM_SPI0_MOSI_IO, PIO14);
		Chip_SWM_MovablePinAssign(SWM_SPI0_MISO_IO, PIO6);
		Chip_SWM_MovablePinAssign(SWM_SPI0_SSEL_IO, PIO13);
		
	}
	else {
		/*
		 * Initialize SSP0 pins connect
		 * SCK1: PINASSIGN4[31:24]: Select P0.12
		 * MOSI1: PINASSIGN5[7:0]: Select P0.14
		 * MISO1: PINASSIGN5[15:8] : Select P0.6
		 * SSEL1: PINASSIGN5[23:16]: Select P0.13
		 */
		Chip_SWM_FixedPinEnable(VDDCMP, DISABLE);
		Chip_SWM_MovablePinAssign(SWM_SPI1_SCK_IO, PIO12);
		Chip_SWM_MovablePinAssign(SWM_SPI1_MOSI_IO, PIO14);
		Chip_SWM_MovablePinAssign(SWM_SPI1_MISO_IO, PIO6);
		Chip_SWM_MovablePinAssign(SWM_SPI1_SSEL_IO, PIO13);
	}
	/* Disable the clock to the Switch Matrix to save power */
	Chip_Clock_DisablePeriphClock(SYSCTL_CLOCK_SWM);
}

/* Initializes clock and pin muxing for I2C interface */
void Board_I2C_Init(void)
{
	/* Enable the clock to the Switch Matrix */
	Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_SWM);
	/* Connect the I2C_SDA and I2C_SCL signals to port pins(P0.10, P0.11) */
	Chip_SWM_MovablePinAssign(SWM_I2C_SDA_IO, PIO10);
	Chip_SWM_MovablePinAssign(SWM_I2C_SCL_IO, PIO11);

	/* Enable Fast Mode Plus for I2C pins */
	Chip_IOCON_PinSetI2CMode(LPC_IOCON, PIO10, PIN_I2CMODE_FASTPLUS);
	Chip_IOCON_PinSetI2CMode(LPC_IOCON, PIO11, PIN_I2CMODE_FASTPLUS);
}

/* Sends a character on the UART */
void Board_UARTPutChar(char ch)
{
#if defined(DEBUG_UART)
	while (Chip_UART_SendByte(DEBUG_UART, (uint8_t) ch) == ERROR) {}
#endif
}

/* Gets a character from the UART, returns EOF if no character is ready */
int Board_UARTGetChar(void)
{
#if defined(DEBUG_UART)
	uint8_t data;

	if (Chip_UART_ReceiveByte(DEBUG_UART, &data) == SUCCESS) {
		return (int) data;
	}
#endif
	return EOF;
}

/* Outputs a string on the debug UART */
void Board_UARTPutSTR(char *str)
{
#if defined(DEBUG_UART)
	while (*str != '\0') {
		Board_UARTPutChar(*str++);
	}
#endif
}

/* Initialize debug output via UART for board */
void Board_Debug_Init(void)
{
#if defined(DEBUG_UART)
	Board_UART_Init(DEBUG_UART);


	Chip_UART_ConfigData(DEBUG_UART, UART_DATALEN_8, UART_PARITY_NONE, UART_STOPLEN_1);
	Chip_UART_SetBaudRate(DEBUG_UART, 115200);
	Chip_UART_Init(DEBUG_UART);
#endif
}

/* Set up and initialize all required blocks and functions related to the
   board hardware */
void Board_Init(void)
{
	/* Initialize the system core clock variable */
	SystemCoreClockUpdate();

	/* Sets up DEBUG UART */
	DEBUGINIT();

	/* Initialize GPIO */
	Chip_GPIO_Init(LPC_GPIO_PORT);

	/* Initialize the LEDs */
	Board_LED_Init();
}

/**
 * @}
 */
