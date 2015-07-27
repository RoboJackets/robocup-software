/*
 * @brief Embedded Artists LPC1788 and LPC4088 Development Kit board file
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

/* EA1788 and EA4088 board uses 8720 PHY and retarget */
#include "lpc_phy_smsc87x0.c"
#include "lpc_norflash_sst39vf320.c"
#include "lpc_nandflash_k9f1g.c"
#include "retarget.c"

/** @ingroup BOARD_EA_DEVKIT_17884088
 * @{
 */

/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/

#if defined(CHIP_LPC177X_8X)
#define LCD_SSP_CTRL                       LPC_SSP0
#elif defined(CHIP_LPC407X_8X)
#define LCD_SSP_CTRL                       LPC_SSP2
#endif

/* No. of TSC conversion bits */
#define TSC2046_CONVERSION_BITS            12

/* Touchscreen TSC2046 control byte definitions */
#define TSC_START                          (0x01 << 7)
#define TSC_CHANNEL_X                      (0x05 << 4)		/* differential */
#define TSC_CHANNEL_Y                      (0x01 << 4)		/* differential */
#define TSC_CHANNEL_Z1                     (0x03 << 4)		/* differential */
#define TSC_CHANNEL_Z2                     (0x04 << 4)		/* differential */
#define TSC_8BIT                           (0x01 << 3)
#define TSC_12BIT                          (0x00 << 3)
#define TSC_PD                              0x00
#define TSC_ADC_ON                          0x01
#define TSC_REF_ON                          0x02
#if (TSC2046_CONVERSION_BITS == 12)
#define TSC_CONVERSION_MODE                 TSC_12BIT
#else
#define TSC_CONVERSION_MODE                 TSC_8BIT
#endif

#define TSC_SER_MODE                        (0x01 << 2)	/* Single-Ended Reference Mode */
#define TSC_DFR_MODE                        (0x00 << 2)	/* Differential Reference Mode */

#define X_MEASURE      (TSC_START | TSC_CHANNEL_X  | TSC_CONVERSION_MODE | TSC_DFR_MODE | TSC_ADC_ON)
#define Y_MEASURE      (TSC_START | TSC_CHANNEL_Y  | TSC_CONVERSION_MODE | TSC_DFR_MODE | TSC_ADC_ON)
#define Z1_MEASURE     (TSC_START | TSC_CHANNEL_Z1 | TSC_CONVERSION_MODE | TSC_DFR_MODE | TSC_ADC_ON)
#define Z2_MEASURE     (TSC_START | TSC_CHANNEL_Z2 | TSC_CONVERSION_MODE | TSC_DFR_MODE | TSC_ADC_ON)
#define PWRDOWN        (TSC_START | TSC_CHANNEL_Y  | TSC_CONVERSION_MODE | TSC_DFR_MODE | TSC_PD)
#if (TSC2046_CONVERSION_BITS == 12)
#define TSC2046_COORD_MAX             (0xFFF)
#define TSC2046_DELTA_VARIANCE        (0x50)
#else
#define TSC2046_COORD_MAX             (0xFF)
#define TSC2046_DELTA_VARIANCE        (0x03)
#endif
#define COORD_GET_NUM                 (10)
#define TSC2046_CS_PORTNUM             0
#define TSC2046_CS_PINNUM              20
#define TSC2046_DC_PORTNUM             0
#define TSC2046_DC_PINNUM              19
#define LCD_DC_CMD      (Chip_GPIO_WritePortBit(LPC_GPIO, TSC2046_DC_PORTNUM, TSC2046_DC_PINNUM, false))
#define LCD_DC_DATA     (Chip_GPIO_WritePortBit(LPC_GPIO, TSC2046_DC_PORTNUM, TSC2046_DC_PINNUM, true))
#define TSC_CS_ON       (Chip_GPIO_WritePortBit(LPC_GPIO, TSC2046_CS_PORTNUM, TSC2046_CS_PINNUM, false))
#define TSC_CS_OFF      (Chip_GPIO_WritePortBit(LPC_GPIO, TSC2046_CS_PORTNUM, TSC2046_CS_PINNUM, true))

#define LCD_BACKLIGHT_PORTNUM          1
#define LCD_BACKLIGHT_PINNUM           18

#define BUTTONS_BUTTON1_GPIO_PORT_NUM           2
#define BUTTONS_BUTTON1_GPIO_BIT_NUM            10
#define JOYSTICK_UP_GPIO_PORT_NUM               2
#define JOYSTICK_UP_GPIO_BIT_NUM                26
#define JOYSTICK_DOWN_GPIO_PORT_NUM             2
#define JOYSTICK_DOWN_GPIO_BIT_NUM              23
#define JOYSTICK_LEFT_GPIO_PORT_NUM             2
#define JOYSTICK_LEFT_GPIO_BIT_NUM              25
#define JOYSTICK_RIGHT_GPIO_PORT_NUM            2
#define JOYSTICK_RIGHT_GPIO_BIT_NUM             27
#define JOYSTICK_PRESS_GPIO_PORT_NUM            2
#define JOYSTICK_PRESS_GPIO_BIT_NUM             22

#define NUM_LEDS 2

typedef struct {
	int16_t ad_left;						/* left margin */
	int16_t ad_right;						/* right margin */
	int16_t ad_top;							/* top margin */
	int16_t ad_bottom;						/* bottom margin */
	int16_t lcd_width;						/* lcd horizontal size */
	int16_t lcd_height;						/* lcd vertical size */
	uint8_t swap_xy;						/* 1: swap x-y coords */
} TSC2046_Init_T;

/** Local variables */
static TSC2046_Init_T TSC_Config = {
	3758, 149, 3914, 163, 240, 320, 1
};

/**
 * LCD configuration data
 */
const LCD_Config_T EA320x240 = {
	28,		/* Horizontal back porch in clocks */
	10,		/* Horizontal front porch in clocks */
	2,		/* HSYNC pulse width in clocks */
	240,	/* Pixels per line */
	2,		/* Vertical back porch in clocks */
	1,		/* Vertical front porch in clocks */
	2,		/* VSYNC pulse width in clocks */
	320,	/* Lines per panel */
	0,		/* Invert output enable, 1 = invert */
	1,		/* Invert panel clock, 1 = invert */
	1,		/* Invert HSYNC, 1 = invert */
	1,		/* Invert VSYNC, 1 = invert */
	1,		/* AC bias frequency in clocks (not used) */
	6,		/* Maximum bits per pixel the display supports */
	LCD_TFT,		/* LCD panel type */
	LCD_COLOR_FORMAT_BGR,		/* BGR or RGB */
	0		/* Dual panel, 1 = dual panel display */
};

/* LEDs on port 2 */
static const uint8_t ledBits[NUM_LEDS] = {26, 27};

/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/

/*!< System Clock Frequency (Core Clock)*/
uint32_t SystemCoreClock;

/*****************************************************************************
 * Private functions
 ****************************************************************************/

/* Very simple (inaccurate) delay function */
static void delay(uint32_t i)
{
	while (i--) {}
}

/* Delay in miliseconds  (cclk = 120MHz) */
static void delayMs(uint32_t ms)
{
	delay(ms * 40000);
}

/* Additional (SPI) pin configuration for LCD interface signals */
/* Write to a LCD register using SPI */
static void writeLCDReg(uint16_t addr, uint16_t data)
{
	uint8_t buf[2];

	LCD_DC_CMD;

	buf[0] = 0;
	buf[1] = (addr & 0xff);

	Chip_SSP_WriteFrames_Blocking(LCD_SSP_CTRL, buf, 2);

	LCD_DC_DATA;
	buf[0] = (data >> 8);
	buf[1] = (data & 0xff);
	Chip_SSP_WriteFrames_Blocking(LCD_SSP_CTRL, buf, 2);

	LCD_DC_CMD;

	buf[0] = (0);
	buf[1] = (0x22);
	Chip_SSP_WriteFrames_Blocking(LCD_SSP_CTRL, buf, 2);
}

/* Initialize SSD1289 LCD Controller */
static void initSSD1289(void)
{
	writeLCDReg(0x00, 0x0001);
	delayMs(15);
	writeLCDReg(0x03, 0x6E3E);	/* 0xAEAC */
	writeLCDReg(0x0C, 0x0007);
	writeLCDReg(0x0D, 0x000E);	/* 0x000F */
	writeLCDReg(0x0E, 0x2C00);	/* 0x2900 */
	writeLCDReg(0x1E, 0x00AE);	/* 0x00B3 */
	delayMs(15);
	writeLCDReg(0x07, 0x0021);
	delayMs(50);
	writeLCDReg(0x07, 0x0023);
	delayMs(50);
	writeLCDReg(0x07, 0x0033);
	delayMs(50);

	writeLCDReg(0x01, 0x2B3F);
	writeLCDReg(0x02, 0x0600);
	writeLCDReg(0x10, 0x0000);
	delayMs(15);
	writeLCDReg(0x11, 0xC5B0);	/* 0x65b0 */
	delayMs(20);
	writeLCDReg(0x05, 0x0000);
	writeLCDReg(0x06, 0x0000);
	writeLCDReg(0x16, 0xEF1C);
	writeLCDReg(0x17, 0x0003);
	writeLCDReg(0x07, 0x0233);
	writeLCDReg(0x0B, 0x5312);
	writeLCDReg(0x0F, 0x0000);
	writeLCDReg(0x25, 0xE000);
	delayMs(20);
	writeLCDReg(0x41, 0x0000);
	writeLCDReg(0x42, 0x0000);
	writeLCDReg(0x48, 0x0000);
	writeLCDReg(0x49, 0x013F);
	writeLCDReg(0x44, 0xEF00);
	writeLCDReg(0x45, 0x0000);
	writeLCDReg(0x46, 0x013F);
	writeLCDReg(0x4A, 0x0000);
	writeLCDReg(0x4B, 0x0000);
	delayMs(20);
	writeLCDReg(0x30, 0x0707);
	writeLCDReg(0x31, 0x0704);
	writeLCDReg(0x32, 0x0005);	/* 0x0204 */
	writeLCDReg(0x33, 0x0402);	/* 0x0201 */
	writeLCDReg(0x34, 0x0203);
	writeLCDReg(0x35, 0x0204);
	writeLCDReg(0x36, 0x0204);
	writeLCDReg(0x37, 0x0401);	/* 0x0502 */
	writeLCDReg(0x3A, 0x0302);
	writeLCDReg(0x3B, 0x0500);
	delayMs(20);
	writeLCDReg(0x22, 0x0000);
}

/* Configure TSC pins */
static void configTSC2046Pins(void)
{
#if defined(CHIP_LPC177X_8X)
	Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 15, (IOCON_FUNC2 | IOCON_MODE_INACT));
	Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 17, (IOCON_FUNC2 | IOCON_MODE_INACT));
	Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 18, (IOCON_FUNC2 | IOCON_MODE_INACT));
#elif defined(CHIP_LPC407X_8X)
	Chip_IOCON_PinMuxSet(LPC_IOCON, 5, 2, (IOCON_FUNC2 | IOCON_MODE_INACT));
	Chip_IOCON_PinMuxSet(LPC_IOCON, 5, 1, (IOCON_FUNC2 | IOCON_MODE_INACT));
	Chip_IOCON_PinMuxSet(LPC_IOCON, 5, 0, (IOCON_FUNC2 | IOCON_MODE_INACT));
#endif

	/*  CS pin */
	Chip_IOCON_PinMuxSet(LPC_IOCON, TSC2046_CS_PORTNUM, TSC2046_CS_PINNUM, (IOCON_FUNC0 | IOCON_MODE_INACT));
	Chip_GPIO_WriteDirBit(LPC_GPIO, TSC2046_CS_PORTNUM, TSC2046_CS_PINNUM, true);
}

/* Send/Receive data to/from TSC2046. */
static void readWriteTSC2046(uint8_t command, uint16_t *data)
{
	uint8_t rx_data[2], tx_data[1] = {0x00};

	tx_data[0] = command;

	TSC_CS_ON;

	Chip_SSP_WriteFrames_Blocking(LCD_SSP_CTRL, tx_data, 1);
	Chip_SSP_ReadFrames_Blocking(LCD_SSP_CTRL, rx_data, 2);

#if (TSC2046_CONVERSION_BITS == 8)
	*data = (((rx_data[0] << 8) | (rx_data[1])) >> 7) & 0xFF;
#else
	*data = (((rx_data[0] << 8) | rx_data[1]) >> 3) & 0xFFF;
#endif

	TSC_CS_OFF;
}

/* Evaluate the coordinates received from TSC2046. */
static Status evalTSC2046Coord(uint8_t command, uint16_t *coord)
{
	uint32_t i;
	uint16_t Tmp = 0, previousTmp;
	int16_t diff = 0;
	*coord = 0;
	for (i = 0; i < COORD_GET_NUM; i++) {
		previousTmp = Tmp;
		readWriteTSC2046(command, &Tmp);
		if (Tmp > TSC2046_COORD_MAX) {
			return ERROR;
		}
		if (i > 0) {
			diff = Tmp - previousTmp;
		}
		if (diff < 0) {
			diff = 0 - diff;
		}
		if (diff > TSC2046_DELTA_VARIANCE) {
			return ERROR;
		}
		*coord += Tmp;
	}
	*coord /= COORD_GET_NUM;
	return SUCCESS;
}

/* Convert the coord received from TSC to a value on truly LCD */
static int16_t calibrateTSC2046(int16_t Coord, int16_t MinVal, int16_t MaxVal, int16_t TrueSize)
{
	int16_t tmp;
	int16_t ret;
	uint8_t convert = 0;

	/* Swap value? */
	if (MinVal > MaxVal) {
		tmp = MaxVal;
		MaxVal = MinVal;
		MinVal = tmp;
		convert = 1;
	}

	ret = (Coord - MinVal) * TrueSize / (MaxVal - MinVal);
	if (convert) {
		ret = TrueSize - ret;
	}

	return ret;
}

/* Initializes board LED(s) */
static void Board_LED_Init(void)
{
	int i;

	/* Setup port direction and initial output state */
	for (i = 0; i < NUM_LEDS; i++) {
		Chip_GPIO_WriteDirBit(LPC_GPIO, 2, ledBits[i], true);
		Chip_GPIO_WritePortBit(LPC_GPIO, 2, ledBits[i], true);
	}
}

/*****************************************************************************
 * Public functions
 ****************************************************************************/

/* Update system core clock rate, should be called if the system has
   a clock rate change */
void SystemCoreClockUpdate(void)
{
	/* CPU core speed */
	SystemCoreClock = Chip_Clock_GetSystemClockRate();
}

/* Initialize UART pins */
void Board_UART_Init(LPC_USART_T *pUART)
{
	if (pUART == LPC_UART0) {
		/*
		 * Initialize UART0 pin connect
		 * P0.2: TXD
		 * P0.3: RXD
		 */
		Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 2, (IOCON_FUNC1 | IOCON_MODE_INACT));
		Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 3, (IOCON_FUNC1 | IOCON_MODE_INACT));
	}
	else if (pUART == LPC_UART2) {
		/* Initialize UART2 pin connect */
#if defined(CHIP_LPC407X_8X)
		Chip_IOCON_PinMuxSet(LPC_IOCON, 4, 22, (IOCON_FUNC2 | IOCON_MODE_INACT));
		Chip_IOCON_PinMuxSet(LPC_IOCON, 4, 23, (IOCON_FUNC2 | IOCON_MODE_INACT));
#else
		Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 10, (IOCON_FUNC1 | IOCON_MODE_INACT));
		Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 11, (IOCON_FUNC1 | IOCON_MODE_INACT));
#endif
	}
}

/* Initialize debug output via UART for board */
void Board_Debug_Init(void)
{
#if defined(DEBUG_UART)
	Board_UART_Init(DEBUG_UART);

	Chip_UART_Init(DEBUG_UART);
	Chip_UART_SetBaud(DEBUG_UART, 115200);
	Chip_UART_ConfigData(DEBUG_UART, UART_DATABIT_8, UART_PARITY_NONE, UART_STOPBIT_1);

	/* Enable UART Transmit */
	Chip_UART_TxCmd(DEBUG_UART, ENABLE);
#endif
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

/* Sets the state of a board LED to on or off */
void Board_LED_Set(uint8_t LEDNumber, bool On)
{
	Chip_GPIO_WritePortBit(LPC_GPIO, 2, ledBits[LEDNumber], (bool) !On);
}

/* Returns the current state of a board LED */
bool Board_LED_Test(uint8_t LEDNumber)
{
	return Chip_GPIO_ReadPortBit(LPC_GPIO, 2, ledBits[LEDNumber]);
}

/* Returns the MAC address assigned to this board */
void Board_ENET_GetMacADDR(uint8_t *mcaddr)
{
	const uint8_t boardmac[] = {0x00, 0x60, 0x37, 0x12, 0x34, 0x56};

	memcpy(mcaddr, boardmac, 6);
}

/* Initialize pin muxing for SSP interface */
void Board_SSP_Init(LPC_SSP_T *pSSP)
{
	if (pSSP == LPC_SSP1) {
		/* Set up clock and muxing for SSP1 interface */
#if defined(CHIP_LPC177X_8X) || defined(CHIP_LPC407X_8X)
		/*
		 * Initialize SSP0 pins connect
		 * P0.7: SCK
		 * P0.6: SSEL
		 * P0.8: MISO
		 * P0.9: MOSI
		 */
		Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 7, (IOCON_FUNC2 | IOCON_MODE_INACT | IOCON_DIGMODE_EN));
		Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 6, (IOCON_FUNC2 | IOCON_MODE_INACT | IOCON_DIGMODE_EN));
		Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 8, (IOCON_FUNC2 | IOCON_MODE_INACT | IOCON_DIGMODE_EN));
		Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 9, (IOCON_FUNC2 | IOCON_MODE_INACT | IOCON_DIGMODE_EN));
#endif
	}
#if !defined(CHIP_LPC175X_6X)
	else if (pSSP == LPC_SSP2) {
#if defined(CHIP_LPC407X_8X)
		/*
		 * Initialize SSP0 pins connect
		 * P5.2: SCK
		 * P5.3: SSEL
		 * P5.1: MISO
		 * P5.0: MOSI
		 */
		Chip_IOCON_PinMuxSet(LPC_IOCON, 5, 2, (IOCON_FUNC2 | IOCON_MODE_INACT));
		Chip_IOCON_PinMuxSet(LPC_IOCON, 5, 3, (IOCON_FUNC2 | IOCON_MODE_INACT));
		Chip_IOCON_PinMuxSet(LPC_IOCON, 5, 1, (IOCON_FUNC2 | IOCON_MODE_INACT));
		Chip_IOCON_PinMuxSet(LPC_IOCON, 5, 0, (IOCON_FUNC2 | IOCON_MODE_INACT));
#endif

	}
#endif
	else {
		/* Set up clock and muxing for SSP0 interface */
#if defined(CHIP_LPC177X_8X)
		/*
		 * Initialize SSP0 pins connect
		 * P0.15: SCK
		 * P0.16: SSEL
		 * P0.17: MISO
		 * P0.18: MOSI
		 */
		Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 15, (IOCON_FUNC2 | IOCON_MODE_INACT));
		Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 16, (IOCON_FUNC2 | IOCON_MODE_INACT));
		Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 17, (IOCON_FUNC2 | IOCON_MODE_INACT));
		Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 18, (IOCON_FUNC2 | IOCON_MODE_INACT));
#endif
	}
}

/* Set up and initialize all required blocks and functions related to the
   board hardware */
void Board_Init(void)
{
	/* Sets up DEBUG UART */
	DEBUGINIT();

	/* Updates SystemCoreClock global var with current clock speed */
	SystemCoreClockUpdate();

	/* Initializes GPIO */
	Chip_GPIO_Init(LPC_GPIO);
	Chip_IOCON_Init(LPC_IOCON);

	/* Initialize LEDs */
	Board_LED_Init();
}

/* Initialize LCD Interface */
void Board_LCD_Init(void)
{
	uint8_t i;

	for (i = 4; i <= 9; i++) {
		Chip_IOCON_PinMuxSet(LPC_IOCON, 0, i, (IOCON_FUNC7 | IOCON_MODE_INACT));
	}
	for (i = 20; i <= 29; i++) {
		Chip_IOCON_PinMuxSet(LPC_IOCON, 1, i, (IOCON_FUNC7 | IOCON_MODE_INACT));
	}
	for (i = 0; i <= 6; i++) {
		Chip_IOCON_PinMuxSet(LPC_IOCON, 2, i, (IOCON_FUNC7 | IOCON_MODE_INACT));
	}
#if defined(CHIP_LPC177X_8X)
	Chip_IOCON_PinMuxSet(LPC_IOCON, 2, 7, (IOCON_FUNC7 | IOCON_MODE_INACT));
#elif defined(CHIP_LPC407X_8X)
	Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 10, (IOCON_FUNC7 | IOCON_MODE_INACT));
#endif
	Chip_IOCON_PinMuxSet(LPC_IOCON, 2, 8, (IOCON_FUNC7 | IOCON_MODE_INACT));
	Chip_IOCON_PinMuxSet(LPC_IOCON, 2, 9, (IOCON_FUNC7 | IOCON_MODE_INACT));

	for (i = 11; i <= 13; i++) {
		Chip_IOCON_PinMuxSet(LPC_IOCON, 2, i, (IOCON_FUNC7 | IOCON_MODE_INACT));
	}
	Chip_IOCON_PinMuxSet(LPC_IOCON, 4, 28, (IOCON_FUNC7 | IOCON_MODE_INACT));
	Chip_IOCON_PinMuxSet(LPC_IOCON, 4, 29, (IOCON_FUNC7 | IOCON_MODE_INACT));
	Chip_IOCON_PinMuxSet(LPC_IOCON, LCD_BACKLIGHT_PORTNUM, LCD_BACKLIGHT_PINNUM, (IOCON_FUNC0 | IOCON_MODE_INACT));
	Chip_GPIO_WriteDirBit(LPC_GPIO, LCD_BACKLIGHT_PORTNUM, LCD_BACKLIGHT_PINNUM, true);

	Board_InitLCDController();
}

/* Initialize the LCD controller on the external QVGA (320x240) TFT LCD*/
void Board_InitLCDController(void)
{
	SSP_ConfigFormat ssp_format;
	/* SSP pin configuration */
#if defined(CHIP_LPC177X_8X)
	Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 15, (IOCON_FUNC2 | IOCON_MODE_INACT));
	Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 16, (IOCON_FUNC2 | IOCON_MODE_INACT));
	Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 17, (IOCON_FUNC2 | IOCON_MODE_INACT));
	Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 18, (IOCON_FUNC2 | IOCON_MODE_INACT));
#elif defined(CHIP_LPC407X_8X)
	Chip_IOCON_PinMuxSet(LPC_IOCON, 5, 2, (IOCON_FUNC2 | IOCON_MODE_INACT));
	Chip_IOCON_PinMuxSet(LPC_IOCON, 5, 3, (IOCON_FUNC2 | IOCON_MODE_INACT));
	Chip_IOCON_PinMuxSet(LPC_IOCON, 5, 1, (IOCON_FUNC2 | IOCON_MODE_INACT));
	Chip_IOCON_PinMuxSet(LPC_IOCON, 5, 0, (IOCON_FUNC2 | IOCON_MODE_INACT));
#endif
	/*  DC pin */
	Chip_IOCON_PinMuxSet(LPC_IOCON, TSC2046_DC_PORTNUM, TSC2046_DC_PINNUM, (IOCON_FUNC0 | IOCON_MODE_INACT));
	Chip_GPIO_WriteDirBit(LPC_GPIO, TSC2046_DC_PORTNUM, TSC2046_DC_PINNUM, true);

	Chip_SSP_Init(LCD_SSP_CTRL);
	Chip_SSP_SetMaster(LCD_SSP_CTRL, true);
	Chip_SSP_SetBitRate(LCD_SSP_CTRL, 1000000);
	ssp_format.frameFormat = SSP_FRAMEFORMAT_SPI;
	ssp_format.bits = SSP_BITS_8;
	ssp_format.clockMode = SSP_CLOCK_MODE0;

	Chip_SSP_SetFormat(LCD_SSP_CTRL, &ssp_format);
	Chip_SSP_Enable(LCD_SSP_CTRL);

	delayMs(200);
	/* initialize LCD controller */
	initSSD1289();

	/* Power of SSP inteface */
	Chip_SSP_Disable(LCD_SSP_CTRL);
	Chip_SSP_DeInit(LCD_SSP_CTRL);

#if defined(CHIP_LPC177X_8X)
	Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 15, (IOCON_FUNC0 | IOCON_MODE_INACT));
	Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 16, (IOCON_FUNC0 | IOCON_MODE_INACT));
	Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 17, (IOCON_FUNC0 | IOCON_MODE_INACT));
	Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 18, (IOCON_FUNC0 | IOCON_MODE_INACT));
#elif defined(CHIP_LPC407X_8X)
	Chip_IOCON_PinMuxSet(LPC_IOCON, 5, 2, (IOCON_FUNC0 | IOCON_MODE_INACT));
	Chip_IOCON_PinMuxSet(LPC_IOCON, 5, 3, (IOCON_FUNC0 | IOCON_MODE_INACT));
	Chip_IOCON_PinMuxSet(LPC_IOCON, 5, 1, (IOCON_FUNC0 | IOCON_MODE_INACT));
	Chip_IOCON_PinMuxSet(LPC_IOCON, 5, 0, (IOCON_FUNC0 | IOCON_MODE_INACT));
#endif
}

/* Initialize TSC2046 touchscreen controller */
void Board_InitTouchController(void)
{
	uint16_t dummy_data;
	SSP_ConfigFormat ssp_format;

	configTSC2046Pins();

	Chip_SSP_Init(LCD_SSP_CTRL);

	Chip_SSP_SetMaster(LCD_SSP_CTRL, true);
	Chip_SSP_SetBitRate(LCD_SSP_CTRL, 200000);

	ssp_format.frameFormat = SSP_FRAMEFORMAT_SPI;
	ssp_format.bits = SSP_BITS_8;
	ssp_format.clockMode = SSP_CLOCK_MODE3;

	Chip_SSP_SetFormat(LCD_SSP_CTRL, &ssp_format);
	Chip_SSP_Enable(LCD_SSP_CTRL);

	/* Enable Touch Screen Controller */
	readWriteTSC2046(PWRDOWN, &dummy_data);
}

/* Get Touch coordinates */
bool Board_GetTouchPos(int16_t *pX, int16_t *pY)
{
	uint16_t tmp;
	uint16_t x, y, z1, z2, z = 0;
	Status Sts = SUCCESS;

	readWriteTSC2046(X_MEASURE, &x);
	readWriteTSC2046(Y_MEASURE, &y);
	readWriteTSC2046(Z1_MEASURE, &z1);
	readWriteTSC2046(Z2_MEASURE, &z2);

	if (z1 != 0) {
		z = x * ((z2 / z1) - 1);
	}
	if ((z <= 0) || (z > 35000)) {
		return false;
	}
	/* Get X-Coordinate */
	Sts = evalTSC2046Coord(X_MEASURE, &x);

	if (Sts == ERROR) {
		return false;
	}
	/* Get Y-Coordinate */
	Sts = evalTSC2046Coord(Y_MEASURE, &y);
	if (Sts == ERROR) {
		return false;
	}
	/* Get Z1-Value */
	Sts = evalTSC2046Coord(Z1_MEASURE, &z1);
	if (Sts == ERROR) {
		return false;
	}
	/* Get Z2-Value */
	Sts = evalTSC2046Coord(Z2_MEASURE, &z2);
	if (Sts == ERROR) {
		return false;
	}

	z = x * ((z2 / z1) - 1);
	if ((z <= 0) || (z > 35000)) {
		return false;
	}
	else {
		/* Swap, adjust to truly size of LCD */
		if (TSC_Config.swap_xy) {
			*pY = calibrateTSC2046(x, TSC_Config.ad_top, TSC_Config.ad_bottom, TSC_Config.lcd_height);
			*pX = calibrateTSC2046(y, TSC_Config.ad_left, TSC_Config.ad_right, TSC_Config.lcd_width);
		}
		else {
			*pX = calibrateTSC2046(x, TSC_Config.ad_top, TSC_Config.ad_bottom, TSC_Config.lcd_width);
			*pY = calibrateTSC2046(y, TSC_Config.ad_left, TSC_Config.ad_right, TSC_Config.lcd_height);
		}
	}
	readWriteTSC2046(PWRDOWN, &tmp);

	return true;
}

/* Turn on Board LCD Backlight */
void Board_SetLCDBacklight(uint8_t Intensity)
{
	bool OnOff = (bool) (Intensity != 0);

	Chip_GPIO_WritePortBit(LPC_GPIO, LCD_BACKLIGHT_PORTNUM, LCD_BACKLIGHT_PINNUM, OnOff);
}

void Board_Audio_Init(LPC_I2S_T *pI2S, int micIn)
{
	Chip_I2S_Audio_Format_T I2S_Config;

	/* Chip_Clock_EnablePeripheralClock(SYSCTL_CLOCK_I2S); */

	I2S_Config.SampleRate = 48000;
	I2S_Config.ChannelNumber = 2;	/* 1 is mono, 2 is stereo */
	I2S_Config.WordWidth =  16;		/* 8, 16 or 32 bits */
	Chip_I2S_Init(pI2S);
	Chip_I2S_Config(pI2S, I2S_TX_MODE, &I2S_Config);

	/* Init UDA1380 CODEC */
	while (!UDA1380_Init(UDA1380_MIC_IN_LR & - (micIn != 0))) {}
}

/* Sets up board specific I2C interface */
void Board_I2C_Init(I2C_ID_T id)
{
	switch (id) {
	case I2C0:
		Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 27, IOCON_FUNC1);
		Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 28, IOCON_FUNC1);
		break;

	case I2C1:
		Chip_IOCON_PinMuxSet(LPC_IOCON, 2, 14, (IOCON_FUNC2 | IOCON_MODE_PULLUP | IOCON_OPENDRAIN_EN));
		Chip_IOCON_PinMuxSet(LPC_IOCON, 2, 15, (IOCON_FUNC2 | IOCON_MODE_PULLUP | IOCON_OPENDRAIN_EN));
		break;

	case I2C2:
		Chip_IOCON_PinMuxSet(LPC_IOCON, 2, 30, (IOCON_FUNC2 | IOCON_MODE_PULLUP | IOCON_OPENDRAIN_EN));
		Chip_IOCON_PinMuxSet(LPC_IOCON, 2, 31, (IOCON_FUNC2 | IOCON_MODE_PULLUP | IOCON_OPENDRAIN_EN));
		break;

	default:
		return;
	}
}

void Board_Buttons_Init(void)
{
	Chip_GPIO_WriteDirBit(LPC_GPIO, BUTTONS_BUTTON1_GPIO_PORT_NUM, BUTTONS_BUTTON1_GPIO_BIT_NUM, false);
}

uint32_t Buttons_GetStatus(void)
{
	uint8_t ret = NO_BUTTON_PRESSED;
	if (Chip_GPIO_ReadPortBit(LPC_GPIO, BUTTONS_BUTTON1_GPIO_PORT_NUM, BUTTONS_BUTTON1_GPIO_BIT_NUM) == 0x00) {
		ret |= BUTTONS_BUTTON1;
	}
	return ret;
}

void Board_Joystick_Init(void)
{
	Chip_IOCON_PinMuxSet(LPC_IOCON, JOYSTICK_UP_GPIO_PORT_NUM, JOYSTICK_UP_GPIO_BIT_NUM,
						 (IOCON_FUNC0 | IOCON_MODE_INACT));
	Chip_IOCON_PinMuxSet(LPC_IOCON, JOYSTICK_DOWN_GPIO_PORT_NUM, JOYSTICK_DOWN_GPIO_BIT_NUM,
						 (IOCON_FUNC0 | IOCON_MODE_INACT));
	Chip_IOCON_PinMuxSet(LPC_IOCON, JOYSTICK_LEFT_GPIO_PORT_NUM, JOYSTICK_LEFT_GPIO_BIT_NUM,
						 (IOCON_FUNC0 | IOCON_MODE_INACT));
	Chip_IOCON_PinMuxSet(LPC_IOCON, JOYSTICK_RIGHT_GPIO_PORT_NUM, JOYSTICK_RIGHT_GPIO_BIT_NUM,
						 (IOCON_FUNC0 | IOCON_MODE_INACT));
	Chip_IOCON_PinMuxSet(LPC_IOCON, JOYSTICK_PRESS_GPIO_PORT_NUM, JOYSTICK_PRESS_GPIO_BIT_NUM,
						 (IOCON_FUNC0 | IOCON_MODE_INACT));
	Chip_GPIO_WriteDirBit(LPC_GPIO, JOYSTICK_UP_GPIO_PORT_NUM, JOYSTICK_UP_GPIO_BIT_NUM, false);		// input
	Chip_GPIO_WriteDirBit(LPC_GPIO, JOYSTICK_DOWN_GPIO_PORT_NUM, JOYSTICK_DOWN_GPIO_BIT_NUM, false);	// input
	Chip_GPIO_WriteDirBit(LPC_GPIO, JOYSTICK_LEFT_GPIO_PORT_NUM, JOYSTICK_LEFT_GPIO_BIT_NUM, false);	// input
	Chip_GPIO_WriteDirBit(LPC_GPIO, JOYSTICK_RIGHT_GPIO_PORT_NUM, JOYSTICK_RIGHT_GPIO_BIT_NUM, false);	// input
	Chip_GPIO_WriteDirBit(LPC_GPIO, JOYSTICK_PRESS_GPIO_PORT_NUM, JOYSTICK_PRESS_GPIO_BIT_NUM, false);	// input
}

/* Sets up board specific CAN interface */
void Board_CAN_Init(LPC_CAN_T *pCAN)
{
	if (pCAN == LPC_CAN1) {
		Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 0, (IOCON_FUNC1 | IOCON_MODE_INACT));
		Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 1, (IOCON_FUNC1 | IOCON_MODE_INACT));
	}
	else {
		Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 4, (IOCON_FUNC2 | IOCON_MODE_INACT));
		Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 5, (IOCON_FUNC2 | IOCON_MODE_INACT));
	}
}

uint8_t Joystick_GetStatus(void)
{
	uint8_t ret = NO_BUTTON_PRESSED;
	if ((Chip_GPIO_ReadPortBit(LPC_GPIO, JOYSTICK_UP_GPIO_PORT_NUM, JOYSTICK_UP_GPIO_BIT_NUM)) == 0x00) {
		ret |= JOY_UP;
	}
	else if (Chip_GPIO_ReadPortBit(LPC_GPIO, JOYSTICK_DOWN_GPIO_PORT_NUM, JOYSTICK_DOWN_GPIO_BIT_NUM) == 0x00) {
		ret |= JOY_DOWN;
	}
	else if ((Chip_GPIO_ReadPortBit(LPC_GPIO, JOYSTICK_LEFT_GPIO_PORT_NUM, JOYSTICK_LEFT_GPIO_BIT_NUM)) == 0x00) {
		ret |= JOY_LEFT;
	}
	else if (Chip_GPIO_ReadPortBit(LPC_GPIO, JOYSTICK_RIGHT_GPIO_PORT_NUM, JOYSTICK_RIGHT_GPIO_BIT_NUM) == 0x00) {
		ret |= JOY_RIGHT;
	}
	else if ((Chip_GPIO_ReadPortBit(LPC_GPIO, JOYSTICK_PRESS_GPIO_PORT_NUM, JOYSTICK_PRESS_GPIO_BIT_NUM)) == 0x00) {
		ret |= JOY_PRESS;
	}
	return ret;
}

/* Pin mux for Event Monitor/Recorder */
void Board_RTC_EV_Init(void)
{
	Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 7, (IOCON_FUNC4 | IOCON_MODE_INACT));
	Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 8, (IOCON_FUNC4 | IOCON_MODE_INACT));
	Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 9, (IOCON_FUNC4 | IOCON_MODE_INACT));
}

/* Create Serial Stream */
void Serial_CreateStream(void *Stream)
{
	/* To be implemented */
}

/* Setup board for SDC interface */
void Board_SDC_Init(void)
{
	Chip_IOCON_PinMuxSet(LPC_IOCON, 1, 2, (IOCON_FUNC2 | IOCON_MODE_INACT));
	Chip_IOCON_PinMuxSet(LPC_IOCON, 1, 3, (IOCON_FUNC2 | IOCON_MODE_INACT));
#ifdef CHIP_LPC407X_8X
	Chip_IOCON_PinMuxSet(LPC_IOCON, 1, 5, (IOCON_FUNC2 | IOCON_MODE_INACT | IOCON_DIGMODE_EN));
	Chip_IOCON_PinMuxSet(LPC_IOCON, 1, 6, (IOCON_FUNC2 | IOCON_MODE_INACT | IOCON_DIGMODE_EN));
	Chip_IOCON_PinMuxSet(LPC_IOCON, 1, 7, (IOCON_FUNC2 | IOCON_MODE_INACT | IOCON_DIGMODE_EN));
#else
	Chip_IOCON_PinMuxSet(LPC_IOCON, 1, 5, (IOCON_FUNC2 | IOCON_MODE_INACT));
	Chip_IOCON_PinMuxSet(LPC_IOCON, 1, 6, (IOCON_FUNC2 | IOCON_MODE_INACT));
	Chip_IOCON_PinMuxSet(LPC_IOCON, 1, 7, (IOCON_FUNC2 | IOCON_MODE_INACT));
#endif
	Chip_IOCON_PinMuxSet(LPC_IOCON, 1, 11, (IOCON_FUNC2 | IOCON_MODE_INACT));
	Chip_IOCON_PinMuxSet(LPC_IOCON, 1, 12, (IOCON_FUNC2 | IOCON_MODE_INACT));
}

#ifdef CHIP_LPC407X_8X
/* Setup board for CMP interface */
void Board_CMP_Init(void)
{
	/* CMP1_IN0 @ P1.7  */
	Chip_IOCON_PinMuxSet(LPC_IOCON, 1, 7, (IOCON_FUNC5 | IOCON_MODE_INACT | IOCON_ADMODE_EN));
}

/* Setup board for SPIFI interface */
void Board_SPIFI_Init(void)
{
	/* SPIFI_IO[2] */
	Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 15, (IOCON_FUNC5 | IOCON_MODE_INACT));
	/* SPIFI_IO[3] */
	Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 16, (IOCON_FUNC5 | IOCON_MODE_INACT));
	/* SPIFI_IO[1] */
	Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 17, (IOCON_FUNC5 | IOCON_MODE_INACT));
	/* SPIFI_IO[0] */
	Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 18, (IOCON_FUNC5 | IOCON_MODE_INACT));
	/* SPIFI_CLK */
	Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 22, (IOCON_FUNC5 | IOCON_MODE_INACT));
	/* SPIFI_CS */
	Chip_IOCON_PinMuxSet(LPC_IOCON, 2, 7, (IOCON_FUNC5 | IOCON_MODE_INACT));
}

#endif

void Board_NANDFLash_Init(void)
{
	/* NANDFLASH Ready pin */
	Chip_IOCON_PinMuxSet(LPC_IOCON, NANDFLASH_READY_PORT, NANDFLASH_READY_PIN, (IOCON_FUNC0 | IOCON_MODE_INACT));
	Chip_GPIO_WriteDirBit(LPC_GPIO, NANDFLASH_READY_PORT, NANDFLASH_READY_PIN, false);
#if defined(NAND_SUPPORTED_LOCKEDCS)
	/* P4.31 is GPIO out for NAND CS, default is low */
	Board_NANDFLash_CSLatch(false);
	Chip_IOCON_PinMuxSet(LPC_IOCON, 4, 31, (IOCON_FUNC0 | IOCON_FASTSLEW_EN));
	Chip_GPIO_WriteDirBit(LPC_GPIO, 4, 31, true);
#endif
}

/**
 * @}
 */
