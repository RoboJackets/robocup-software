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

#include "board.h"
#include "string.h"

/* Keil board uses 83848 PHY and retarget */
#include "lpc_phy_dp83848.c"
#include "retarget.c"

/** @ingroup BOARD_KEIL_MCB_18574357
 * @{
 */

/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/
/* Port and bit mapping for LEDs on GPIOs */
static const uint8_t ledports[] = {6, 6, 6, 6, 6, 4, 4, 4};
static const uint8_t ledbits[] = {24, 25, 26, 27, 28, 12, 13, 14};

/** Private definitions for LCD */
#define LCD_CS(x) ((x) ? (Chip_GPIO_WritePortBit(LPC_GPIO_PORT, 7, 16, true)) : (Chip_GPIO_WritePortBit(LPC_GPIO_PORT, \
																										7, 16, false)))

/** Private variables for LCD */
uint32_t g_isPenDn;
uint32_t g_isNewPenDn;
const int32_t ad_left = 3813;
const int32_t ad_top = 3805;// 237;
const int32_t ad_right = 360;
const int32_t ad_bottom = 237;	// 3805;

const LCD_Config_T MCB4300_LCD = {
	8,						/*!< Horizontal back porch in clocks */
	4,						/*!< Horizontal front porch in clocks */
	4,						/*!< HSYNC pulse width in clocks */
	240,					/*!< Pixels per line */
	4,						/*!< Vertical back porch in clocks */
	3,						/*!< Vertical front porch in clocks */
	4,						/*!< VSYNC pulse width in clocks */
	320,					/*!< Lines per panel */
	0,						/*!< Invert output enable, 1 = invert */
	1,						/*!< Invert panel clock, 1 = invert */
	1,						/*!< Invert HSYNC, 1 = invert */
	1,						/*!< Invert VSYNC, 1 = invert */
	1,						/*!< AC bias frequency in clocks (not used) */
	6,						/*!< Maximum bits per pixel the display supports */
	LCD_TFT,				/*!< LCD panel type */
	LCD_COLOR_FORMAT_RGB,	/*!< BGR or RGB */
	0						/*!< Dual panel, 1 = dual panel display */
};

/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/

/*!< System Clock Frequency (Core Clock)*/
uint32_t SystemCoreClock;

/*****************************************************************************
 * Private functions
 ****************************************************************************/

/* Very simple (inaccurate) delay function */
static void delay(uint32_t i) {
	while (i--) {}
}

/* Initializes default settings for UDA1380 */
static Status Board_Audio_CodecInit(int micIn)
{
	/* Reset UDA1380 on board Keil */
	Chip_SCU_PinMuxSet(0x8, 0, (SCU_MODE_MODE_INACT | SCU_MODE_FUNC0));
	Chip_GPIO_WriteDirBit(LPC_GPIO_PORT, 4, 0, true);
	Chip_GPIO_WritePortBit(LPC_GPIO_PORT, 4, 0, true);
	/* delay 1us */
	delay(100000);
	Chip_GPIO_WritePortBit(LPC_GPIO_PORT, 4, 0, false);
	delay(100000);

	if (!UDA1380_Init(UDA1380_MIC_IN_LR & - (micIn != 0))) {
		return ERROR;
	}

	return SUCCESS;
}

/** Private functions for LCD controller */
/* Write to LCD controller, with A0 = 0 */
static void writeLCD00_16(uint16_t c) {

	uint8_t buf[1];
	LCD_CS(0);
	buf[0] = 0x70;
	Chip_SSP_WriteFrames_Blocking(SSP_ID, buf, 1);	// Start + WR Register
	buf[0] = (uint8_t) (c);
	Chip_SSP_WriteFrames_Blocking(SSP_ID, buf, 1);
	LCD_CS(1);
}

/* Write to LCD controller, with A0 = 1 */
static void writeLCD01_16(uint16_t c) {

	uint8_t buf[1];
	LCD_CS(0);
	buf[0] = 0x72;
	Chip_SSP_WriteFrames_Blocking(SSP_ID, buf, 1);	/* Start + WR Data */
	buf[0] = (uint8_t) (c >> 8);
	Chip_SSP_WriteFrames_Blocking(SSP_ID, buf, 1);
	buf[0] = (uint8_t) (c);
	Chip_SSP_WriteFrames_Blocking(SSP_ID, buf, 1);
	LCD_CS(1);
}

/* Write to LCD controller's register */
static void writeLCDReg(uint16_t reg, uint16_t dat) {
	writeLCD00_16(reg);
	writeLCD01_16(dat);
}

/* Pin configuration to communicate with LCD Controller */
static void pinConfig(void)
{
	/* (DC) */
	Chip_GPIO_WriteDirBit(LPC_GPIO_PORT, 7, 16, true);
}

/* Writes a value to the STMPE811 register */
static uint32_t writeTSCReg(uint8_t regAddr, uint8_t value)
{
	uint8_t val[2];
	val[0] = regAddr;
	val[1] = value;
	return Chip_I2C_MasterSend(TSC_I2C_BUS, TSC_I2C_ADDR, val, sizeof(val));
}

/* Reads a value to the STMPE811 register */
static uint32_t readTSCReg(uint8_t regAddr, uint8_t *value)
{
	return Chip_I2C_MasterCmdRead(TSC_I2C_BUS, TSC_I2C_ADDR, regAddr, value, 1);
}

/* Check if touch is detected or not */
static bool detectTSCTouch(void)
{
	uint8_t  CtrRegVal = 0;

	if (readTSCReg(TSC_CTRL, &CtrRegVal) == 1) {
		if (CtrRegVal & (1 << 7)) {
			return true;
		}
	}
	return false;
}

/* Get the touch coordinates from STMPE811 registers */
static Status getTSCCoord(int16_t *x, int16_t *y)
{
	uint8_t fifo_size, tscData[4], i;

	/* Read all samples except the last one */
	readTSCReg(FIFO_SIZE, &fifo_size);
	for (i = 0; i < fifo_size; ++i) {
		if (Chip_I2C_MasterCmdRead(TSC_I2C_BUS, TSC_I2C_ADDR, DATA_XYZ, tscData, 4) == 0) {
			return ERROR;
		}
	}

	/* Retrieve last taken sample */
	Chip_I2C_MasterCmdRead(TSC_I2C_BUS, TSC_I2C_ADDR, DATA_XYZ, tscData, 4);
	*x = (tscData[0] << 4) | ((tscData[1] & 0xF0) >> 4);
	*y = ((tscData[1] & 0x0F) << 8) | tscData[2];

	/* Clear interrupt flags */
	writeTSCReg(INT_STA, 0x1F);

	return SUCCESS;
}

/*****************************************************************************
 * Public functions
 ****************************************************************************/

/* Update system core clock rate, should be called if the system has
   a clock rate change */
void SystemCoreClockUpdate(void)
{
	/* CPU core speed */
	SystemCoreClock = Chip_Clock_GetRate(CLK_MX_MXCORE);
}

/* Initialize UART pins */
void Board_UART_Init(LPC_USART_T *pUART)
{
	if (pUART == LPC_USART0) {
		Chip_SCU_PinMuxSet(0x2, 0, (SCU_MODE_MODE_REPEATER | SCU_MODE_FUNC1));		/* P2.0 : UART0_TXD */
		Chip_SCU_PinMuxSet(0x2, 1, (SCU_MODE_MODE_PULLUP | SCU_MODE_INBUFF_EN | SCU_MODE_ZIF_DIS | SCU_MODE_FUNC1));	/* P2.1 : UART0_RXD */
	}
	else if (pUART == LPC_USART3) {
		Chip_SCU_PinMuxSet(0x2, 3, (SCU_MODE_MODE_REPEATER | SCU_MODE_FUNC2));			/* P2.3 : UART3_TXD */
		Chip_SCU_PinMuxSet(0x2, 4, (SCU_MODE_MODE_PULLUP | SCU_MODE_INBUFF_EN | SCU_MODE_ZIF_DIS | SCU_MODE_FUNC2));	/* P2.4 : UART3_RXD */
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

/* Initializes board LED(s) */
static void Board_LED_Init()
{
	int i;

	/* Must make sure J21 is installed to enabled LEDs */
	/* PD.10 : LED 0 (leftmost) */
	/* PD.11 : LED 1 */
	/* PD.12 : LED 2 */
	/* PD.13 : LED 3 */
	/* PD.14 : LED 4 */
	/* P9.0  : LED 5 */
	/* P9.1  : LED 6 */
	/* P9.2  : LED 7 (rightmost) */
	for (i = 0; i < (sizeof(ledports) / sizeof(ledports[0])); i++) {
		Chip_GPIO_WriteDirBit(LPC_GPIO_PORT, ledports[i], ledbits[i], true);
	}
}

#ifndef BOARD_LED_TEST_FUNCTION_WORKS
/* FIXME: temporary code for toggle LED support only */
static uint8_t LEDStates;		/* shadow variable for LED states */
#endif

/* Sets the state of a board LED to on or off */
void Board_LED_Set(uint8_t LEDNumber, bool On)
{
	if (LEDNumber <= 7) {
		Chip_GPIO_WritePortBit(LPC_GPIO_PORT, ledports[LEDNumber], ledbits[LEDNumber], On);
#ifndef BOARD_LED_TEST_FUNCTION_WORKS
		if (On) {
			LEDStates |= (1 << LEDNumber);	/* set the state */
		}
		else {
			LEDStates &= ~(1 << LEDNumber);	/* clear the state */
		}
#endif
	}
}

/* Returns the current state of a board LED */
bool Board_LED_Test(uint8_t LEDNumber)
{
	if (LEDNumber <= 7) {
#ifndef BOARD_LED_TEST_FUNCTION_WORKS
		if (LEDStates & (1 << LEDNumber)) {	/* LED is on */
			return true;
		}
		else {	/* LED is off */
			return false;
		}
#else
		return (bool) Chip_GPIO_ReadPortBit(ledports[LEDNumber], ledbits[LEDNumber]);
#endif
	}
	return false;
}

/* Returns the MAC address assigned to this board */
void Board_ENET_GetMacADDR(uint8_t *mcaddr)
{
	const uint8_t boardmac[] = {0x00, 0x60, 0x37, 0x12, 0x34, 0x56};

	memcpy(mcaddr, boardmac, 6);
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
	Chip_GPIO_Init(LPC_GPIO_PORT);

	/* Setup GPIOs for USB demos */
	Chip_SCU_PinMuxSet(0x9, 5, (SCU_MODE_MODE_INACT | SCU_MODE_INBUFF_EN | SCU_MODE_FUNC2));			/* P9_5 USB1_VBUS_EN, USB1 VBus function */
	Chip_SCU_PinMuxSet(0x2, 5, (SCU_MODE_MODE_PULLUP | SCU_MODE_INBUFF_EN | SCU_MODE_ZIF_DIS | SCU_MODE_FUNC2));	/* P2_5 USB1_VBUS, MUST CONFIGURE THIS SIGNAL FOR USB1 NORMAL OPERATION */
	Chip_SCU_PinMuxSet(0x6, 3, (SCU_MODE_MODE_INACT | SCU_MODE_INBUFF_EN | SCU_MODE_FUNC1));			/* P6_3 USB0_PWR_EN, USB0 VBus function */

	/* Initialize LEDs */
	Board_LED_Init();
}

/* Sets up board specific ADC interface */
void Board_ADC_Init(void)
{}

/* Sets up board specific I2C interface */
void Board_I2C_Init(I2C_ID_T id)
{
	if (id == I2C1) {
		/* Configure pin function for I2C1 on PE.13 (I2C1_SDA) and PE.15 (I2C1_SCL) */
		Chip_SCU_PinMuxSet(0xE, 13, (SCU_MODE_ZIF_DIS | SCU_MODE_INBUFF_EN | SCU_MODE_FUNC2));
		Chip_SCU_PinMuxSet(0xE, 15, (SCU_MODE_ZIF_DIS | SCU_MODE_INBUFF_EN | SCU_MODE_FUNC2));
	}
	else {
		Chip_SCU_I2C0PinConfig(I2C0_STANDARD_FAST_MODE);
	}
}

/* Sets up board specific I2S interface and UDA1380 */
void Board_Audio_Init(LPC_I2S_T *pI2S, int micIn)
{
	Chip_I2S_Audio_Format_T I2S_Config;

	I2S_Config.SampleRate = 48000;
	I2S_Config.ChannelNumber = 2;	/* 1 is mono, 2 is stereo */
	I2S_Config.WordWidth =  16;		/* 8, 16 or 32 bits */
	Chip_I2S_Init(LPC_I2S0);
	Chip_I2S_Config(LPC_I2S0, I2S_TX_MODE, &I2S_Config);

	/* Init UDA1380 CODEC */
	while (Board_Audio_CodecInit(micIn) != SUCCESS) {}
}

/* Initialize the LCD interface */
void Board_LCD_Init(void)
{
	/* Reset LCD and wait for reset to complete */
	Chip_RGU_TriggerReset(RGU_LCD_RST);
	while (Chip_RGU_InReset(RGU_LCD_RST)) {}

	/* Set backlight GPIO as an output */
	Chip_GPIO_WriteDirBit(LPC_GPIO_PORT, 3, 8, true);

	Board_InitLCDController();
}

/* Initialize the LCD controller on the external QVGA (320x240) TFT LCD*/
void Board_InitLCDController(void)
{
	/* LCD with HX8347-D LCD Controller                                         */
	SSP_ConfigFormat ssp_format;

	delay(5);

	pinConfig();
	/* TBD Externally */
	Chip_SSP_Init(SSP_ID);
	//	NVIC_EnableIRQ(SSP_ID);
	Chip_SSP_SetMaster(SSP_ID, true);
	Chip_SSP_SetBitRate(SSP_ID, 1000000);

	ssp_format.frameFormat = SSP_FRAMEFORMAT_SPI;
	ssp_format.bits = SSP_BITS_8;
	ssp_format.clockMode = SSP_CLOCK_MODE0;

	Chip_SSP_SetFormat(SSP_ID, &ssp_format);
	Chip_SSP_Enable(SSP_ID);

	delay(200);

	/* Driving ability settings ------------------------------------------------*/
	writeLCDReg(0xEA, 0x00);					/* Power control internal used (1)    */
	writeLCDReg(0xEB, 0x20);					/* Power control internal used (2)    */
	writeLCDReg(0xEC, 0x0C);					/* Source control internal used (1)   */
	writeLCDReg(0xED, 0xC7);					/* Source control internal used (2)   */
	writeLCDReg(0xE8, 0x38);					/* Source output period Normal mode   */
	writeLCDReg(0xE9, 0x10);					/* Source output period Idle mode     */
	writeLCDReg(0xF1, 0x01);					/* RGB 18-bit interface ;0x0110       */
	writeLCDReg(0xF2, 0x10);

	/* Adjust the Gamma Curve --------------------------------------------------*/
	writeLCDReg(0x40, 0x01);
	writeLCDReg(0x41, 0x00);
	writeLCDReg(0x42, 0x00);
	writeLCDReg(0x43, 0x10);
	writeLCDReg(0x44, 0x0E);
	writeLCDReg(0x45, 0x24);
	writeLCDReg(0x46, 0x04);
	writeLCDReg(0x47, 0x50);
	writeLCDReg(0x48, 0x02);
	writeLCDReg(0x49, 0x13);
	writeLCDReg(0x4A, 0x19);
	writeLCDReg(0x4B, 0x19);
	writeLCDReg(0x4C, 0x16);

	writeLCDReg(0x50, 0x1B);
	writeLCDReg(0x51, 0x31);
	writeLCDReg(0x52, 0x2F);
	writeLCDReg(0x53, 0x3F);
	writeLCDReg(0x54, 0x3F);
	writeLCDReg(0x55, 0x3E);
	writeLCDReg(0x56, 0x2F);
	writeLCDReg(0x57, 0x7B);
	writeLCDReg(0x58, 0x09);
	writeLCDReg(0x59, 0x06);
	writeLCDReg(0x5A, 0x06);
	writeLCDReg(0x5B, 0x0C);
	writeLCDReg(0x5C, 0x1D);
	writeLCDReg(0x5D, 0xCC);

	/* Power voltage setting ---------------------------------------------------*/
	writeLCDReg(0x1B, 0x1B);
	writeLCDReg(0x1A, 0x01);
	writeLCDReg(0x24, 0x2F);
	writeLCDReg(0x25, 0x57);
	writeLCDReg(0x23, 0x88);

	/* Power on setting --------------------------------------------------------*/
	writeLCDReg(0x18, 0x36);					/* Internal oscillator frequency adj  */
	writeLCDReg(0x19, 0x01);					/* Enable internal oscillator         */
	writeLCDReg(0x01, 0x00);					/* Normal mode, no scrool             */
	writeLCDReg(0x1F, 0x88);					/* Power control 6 - DDVDH Off        */
	delay(20);
	writeLCDReg(0x1F, 0x82);					/* Power control 6 - Step-up: 3 x VCI */
	delay(5);
	writeLCDReg(0x1F, 0x92);					/* Power control 6 - Step-up: On      */
	delay(5);
	writeLCDReg(0x1F, 0xD2);					/* Power control 6 - VCOML active     */
	delay(5);

	/* Color selection ---------------------------------------------------------*/
	writeLCDReg(0x17, 0x55);					/* RGB, System interface: 16 Bit/Pixel*/
	writeLCDReg(0x00, 0x00);					/* Scrolling off, no standby          */

	/* Interface config --------------------------------------------------------*/
	writeLCDReg(0x2F, 0x11);					/* LCD Drive: 1-line inversion        */
	writeLCDReg(0x31, 0x02);					/* Value for SPI: 0x00, RBG: 0x02     */
	writeLCDReg(0x32, 0x00);					/* DPL=0, HSPL=0, VSPL=0, EPL=0       */

	/* Display on setting ------------------------------------------------------*/
	writeLCDReg(0x28, 0x38);					/* PT(0,0) active, VGL/VGL            */
	delay(20);
	writeLCDReg(0x28, 0x3C);					/* Display active, VGL/VGL            */

	writeLCDReg(0x16, 0x00);					/* Mem Access Control (MX/Y/V/L,BGR)  */

	/* Display scrolling settings ----------------------------------------------*/
	writeLCDReg(0x0E, 0x00);					/* TFA MSB                            */
	writeLCDReg(0x0F, 0x00);					/* TFA LSB                            */
	writeLCDReg(0x10, 320 >> 8);				/* VSA MSB                            */
	writeLCDReg(0x11, 320 &  0xFF);			/* VSA LSB                            */
	writeLCDReg(0x12, 0x00);					/* BFA MSB                            */
	writeLCDReg(0x13, 0x00);					/* BFA LSB                            */

}

/* Initializes the STMPE811 touch screen controller */
void Board_InitTouchController(void)
{
	volatile int32_t i;
	I2C_EVENTHANDLER_T old = Chip_I2C_GetMasterEventHandler(UDA1380_I2C_BUS);

	/* Initialize Pin mux and other board related I2C stuff */
	Board_I2C_Init(TSC_I2C_BUS);

	/* Init I2C */
	Chip_I2C_Init(TSC_I2C_BUS);
	Chip_I2C_SetClockRate(TSC_I2C_BUS, 100000);
	Chip_I2C_SetMasterEventHandler(TSC_I2C_BUS, Chip_I2C_EventHandlerPolling);

	/* Reset Touch-screen controller	    */
	writeTSCReg(SYS_CTRL1, 0x02);

	for (i = 0; i < 200000; i++) {}

	/* Enable TSC and ADC                 */
	writeTSCReg(SYS_CTRL2, 0x0C);
	/* Enable Touch detect, FIFO          */
	writeTSCReg(INT_EN, 0x07);
	/* Set sample time , 12-bit mode      */
	writeTSCReg(ADC_CTRL1, 0x69);

	for (i = 0; i < 40000; i++) {}

	/* ADC frequency 3.25 MHz             */
	writeTSCReg(ADC_CTRL2, 0x01);
	/* Tracking index: 8, operation mode : XY only */
	writeTSCReg(TSC_CTRL, 0x22);
	/* Detect delay 10us Settle time 500us*/
	writeTSCReg(TSC_CFG, 0xC2);
	/* Threshold for FIFO                 */
	writeTSCReg(FIFO_TH, 0x01);
	/* FIFO reset                         */
	writeTSCReg(FIFO_STA, 0x01);
	/* FIFO not reset	                   */
	writeTSCReg(FIFO_STA, 0x00);
	/* Drive 50 mA typical	              */
	writeTSCReg(TSC_I_DRIVE, 0x01);
	/* Pins are used for touchscreen	    */
	writeTSCReg(GPIO_ALT_FUNCT, 0x00);
	/* Enable TSC	                       */
	writeTSCReg(TSC_CTRL, 0x01);
	/* Clear interrupt status	           */
	writeTSCReg(INT_STA, 0xFF);
	/* Restore I2C to its default mode */
	Chip_I2C_SetMasterEventHandler(TSC_I2C_BUS, old);
}

/* Poll for Touch position */
bool Board_GetTouchPos(int16_t *pX, int16_t *pY)
{
	bool res;
	I2C_EVENTHANDLER_T old = Chip_I2C_GetMasterEventHandler(UDA1380_I2C_BUS);

	/* It is very unlikely that this will loop more than once */
	while (!Chip_I2C_SetMasterEventHandler(TSC_I2C_BUS, Chip_I2C_EventHandlerPolling)) {}
	res = Board_I2C_GetTouchPos(pX, pY);
	/* It is very unlikely that this will loop more than once */
	while (!Chip_I2C_SetMasterEventHandler(TSC_I2C_BUS, old)) {}
	return res;
}

/* Get touch screen position */
bool Board_I2C_GetTouchPos(int16_t *pX, int16_t *pY)
{
	int16_t x, y, rng;
	if (detectTSCTouch()) {
		getTSCCoord(&x, &y);
		g_isPenDn = 1;
		g_isNewPenDn = 1;

		/* calibrate X */
		rng = ad_right - ad_left;
		if (rng < 0) {
			rng = -rng;
		}
		x -= (ad_right < ad_left) ? ad_right : ad_left;
		*pX = (x * C_GLCD_H_SIZE) / rng;
		if (ad_left > ad_right) {
			*pX = C_GLCD_H_SIZE - *pX;
		}

		/* calibrate Y */
		rng = ad_bottom - ad_top;
		if (rng < 0) {
			rng = -rng;
		}
		y -= (ad_bottom < ad_top) ? ad_bottom : ad_top;
		*pY = (y * C_GLCD_V_SIZE) / rng;
		if (ad_top > ad_bottom) {
			*pY = C_GLCD_V_SIZE - *pY;
		}
	}
	else {
		g_isPenDn = 0;
	}

	if (g_isNewPenDn) {
		g_isNewPenDn = 0;
		if (*pX < 0) {
			*pX = -*pX;
		}
		if (*pY < 0) {
			*pY = -*pY;
		}
		return true;
	}
	return false;
}

/* Turn on LCD backlight */
void Board_SetLCDBacklight(uint8_t Intensity)
{
	bool OnOff = (bool) (Intensity != 0);

	Chip_GPIO_WritePortBit(LPC_GPIO_PORT, 3, 8, OnOff);
}

#ifndef CORE_M0
/* PIN0 Interrupt not available in M0 core */
/* Interrupt handler for GPIO0 */
void GPIO0_IRQHandler(void)
{
	static bool On;

	if (Chip_GPIO_IntGetStatus(LPC_GPIO_PIN_INT, 0, 0, 0)) {
		Chip_GPIO_IntClear(LPC_GPIO_PIN_INT, 0, 0);
		On = (bool) !On;
		Board_LED_Set(1, On);
	}
}

/* Initializes board specific GPIO Interrupt */
void Board_GPIO_Int_Init()
{
	Chip_SCU_PinMuxSet(0xF, 9, (SCU_MODE_MODE_PULLUP | SCU_MODE_INBUFF_EN | SCU_MODE_ZIF_DIS | SCU_MODE_FUNC0));	/* PF.9 : POTI button */
	Chip_GPIO_WriteDirBit(LPC_GPIO_PORT, 7, 23, false);	/* PF.9 -> GPIO7[23] : input */
	Chip_SCU_GPIOIntPinSel(0, 7, 23);
	Chip_GPIO_IntCmd(LPC_GPIO_PIN_INT, 0, 0, GPIOPININT_FALLING_EDGE);	/* Configure GPIO0[7] to interrupt pin (SW2 switch) */

	NVIC_EnableIRQ(PIN_INT0_IRQn);	/* enable GPIO interrupt 0 */
}

#endif

/* Initializes SDMMC interface */
void Board_SDMMC_Init(void)
{
	Chip_SCU_PinMuxSet(0xC, 4, (SCU_PINIO_FAST | SCU_MODE_FUNC7));	/* PC.4 connected to SDIO_D0 */
	Chip_SCU_PinMuxSet(0xC, 5, (SCU_PINIO_FAST | SCU_MODE_FUNC7));	/* PC.5 connected to SDIO_D1 */
	Chip_SCU_PinMuxSet(0xC, 6, (SCU_PINIO_FAST | SCU_MODE_FUNC7));	/* PC.6 connected to SDIO_D2 */
	Chip_SCU_PinMuxSet(0xC, 7, (SCU_PINIO_FAST | SCU_MODE_FUNC7));	/* PC.7 connected to SDIO_D3 */

	Chip_SCU_PinMuxSet(0xC, 8, (SCU_MODE_MODE_PULLUP | SCU_MODE_INBUFF_EN | SCU_MODE_FUNC7));	/* PC.4 connected to SDIO_CD */
	Chip_SCU_PinMuxSet(0xC, 10, (SCU_PINIO_FAST | SCU_MODE_FUNC7));	/* PC.10 connected to SDIO_CMD */
	Chip_SCU_PinMuxSet(0xC, 0, (SCU_MODE_MODE_PULLUP | SCU_MODE_HIGHSPEEDSLEW_EN | SCU_MODE_FUNC7));/* PC.0 connected to SDIO_CLK */
}

/* Initializes SSP interface */
void Board_SSP_Init(LPC_SSP_T *pSSP)
{
	if (pSSP == LPC_SSP0) {
		/* Set up clock and power for SSP0 module */
		/* Configure SSP0 pins*/
		Chip_SCU_PinMuxSet(0x3, 3, (SCU_PINIO_FAST | SCU_MODE_FUNC2));	/* P3.3 connected to SCL/SCLK	SCU_MODE_FUNC2=SSP0 SCK0 */
		Chip_SCU_PinMuxSet(0x3, 6, (SCU_PINIO_FAST | SCU_MODE_FUNC2));	/* P3.6 connected to nCS		SCU_MODE_FUNC2=SSP0 SSEL0 */
		Chip_SCU_PinMuxSet(0x3, 7, (SCU_MODE_MODE_PULLUP | SCU_MODE_INBUFF_EN | SCU_MODE_ZIF_DIS | SCU_MODE_FUNC2));	/* P3.7 connected to SO		SCU_MODE_FUNC2=SSP0 MISO0 */
		Chip_SCU_PinMuxSet(0x3, 8, (SCU_MODE_MODE_PULLUP | SCU_MODE_INBUFF_EN | SCU_MODE_ZIF_DIS | SCU_MODE_FUNC2));	/* P3.8 connected to nSI	SCU_MODE_FUNC2=SSP0 MOSI0 */
	}
	else if (pSSP == LPC_SSP1) {
		/* Set up clock and power for SSP1 module */
		/* Configure SSP1 pins*/
		Chip_SCU_PinMuxSet(0xF, 4, (SCU_PINIO_FAST | SCU_MODE_FUNC0));	/* PF.4 connected to SCL/SCLK	SCU_MODE_FUNC0 = SSP1 SCK1 */
		Chip_SCU_PinMuxSet(0xF, 5, (SCU_PINIO_FAST | SCU_MODE_FUNC2));	/* PF.5 connected to nCS		SCU_MODE_FUNC2 = SSP1 SSEL1 */
		Chip_SCU_PinMuxSet(0xF, 6, (SCU_MODE_MODE_PULLUP | SCU_MODE_INBUFF_EN | SCU_MODE_ZIF_DIS | SCU_MODE_FUNC2));	/* PF.6 connected to SO			  SCU_MODE_FUNC2 = SSP1 MISO1 */
		Chip_SCU_PinMuxSet(0xF, 7, (SCU_MODE_MODE_PULLUP | SCU_MODE_INBUFF_EN | SCU_MODE_ZIF_DIS | SCU_MODE_FUNC2));	/* PF.7 connected to nSI		  SCU_MODE_FUNC2 = SSP1 MOSI1 */
	}
	else {
		return;
	}
}

/* Initializes board specific buttons */
void Board_Buttons_Init(void)
{
	Chip_SCU_PinMuxSet(0x4,  0, (SCU_MODE_MODE_INACT | SCU_MODE_INBUFF_EN | SCU_MODE_ZIF_DIS | SCU_MODE_FUNC0));	/* P9.1  : LED 6 */
	Chip_GPIO_WriteDirBit(LPC_GPIO_PORT, BUTTONS_BUTTON1_GPIO_PORT_NUM, BUTTONS_BUTTON1_GPIO_BIT_NUM, false);
}

/* Sets up default states for joystick */
void Board_Joystick_Init(void)
{
	Chip_SCU_PinMuxSet(0xC, 9, (SCU_MODE_MODE_INACT | SCU_MODE_INBUFF_EN | SCU_MODE_ZIF_DIS | SCU_MODE_FUNC4));		/* PC_9  as GPIO6[8] */
	Chip_SCU_PinMuxSet(0xC, 11, (SCU_MODE_MODE_INACT | SCU_MODE_INBUFF_EN | SCU_MODE_ZIF_DIS | SCU_MODE_FUNC4));		/* PC_11 as GPIO6[10] */
	Chip_SCU_PinMuxSet(0xC, 12, (SCU_MODE_MODE_INACT | SCU_MODE_INBUFF_EN | SCU_MODE_ZIF_DIS | SCU_MODE_FUNC4));		/* PC_12 as GPIO6[11] */
	Chip_SCU_PinMuxSet(0xC, 13, (SCU_MODE_MODE_INACT | SCU_MODE_INBUFF_EN | SCU_MODE_ZIF_DIS | SCU_MODE_FUNC4));		/* PC_13 as GPIO6[12] */
	Chip_SCU_PinMuxSet(0xC, 14, (SCU_MODE_MODE_INACT | SCU_MODE_INBUFF_EN | SCU_MODE_ZIF_DIS | SCU_MODE_FUNC4));		/* PC_14 as GPIO6[13] */

	Chip_GPIO_WriteDirBit(LPC_GPIO_PORT, JOYSTICK_UP_GPIO_PORT_NUM, JOYSTICK_UP_GPIO_BIT_NUM, false);		/* input */
	Chip_GPIO_WriteDirBit(LPC_GPIO_PORT, JOYSTICK_DOWN_GPIO_PORT_NUM, JOYSTICK_DOWN_GPIO_BIT_NUM, false);	/* input */
	Chip_GPIO_WriteDirBit(LPC_GPIO_PORT, JOYSTICK_LEFT_GPIO_PORT_NUM, JOYSTICK_LEFT_GPIO_BIT_NUM, false);	/* input */
	Chip_GPIO_WriteDirBit(LPC_GPIO_PORT, JOYSTICK_RIGHT_GPIO_PORT_NUM, JOYSTICK_RIGHT_GPIO_BIT_NUM, false);	/* input */
	Chip_GPIO_WriteDirBit(LPC_GPIO_PORT, JOYSTICK_PRESS_GPIO_PORT_NUM, JOYSTICK_PRESS_GPIO_BIT_NUM, false);	/* input */
}

/* Gets joystick status */
uint8_t Joystick_GetStatus(void)
{

	uint8_t ret = NO_BUTTON_PRESSED;

	if (Chip_GPIO_ReadPortBit(LPC_GPIO_PORT, JOYSTICK_UP_GPIO_PORT_NUM, JOYSTICK_UP_GPIO_BIT_NUM) == 0) {
		ret |= JOY_UP;
	}
	else if (Chip_GPIO_ReadPortBit(LPC_GPIO_PORT, JOYSTICK_DOWN_GPIO_PORT_NUM, JOYSTICK_DOWN_GPIO_BIT_NUM) == 0) {
		ret |= JOY_DOWN;
	}
	else if (Chip_GPIO_ReadPortBit(LPC_GPIO_PORT, JOYSTICK_LEFT_GPIO_PORT_NUM, JOYSTICK_LEFT_GPIO_BIT_NUM) == 0) {
		ret |= JOY_LEFT;
	}
	else if (Chip_GPIO_ReadPortBit(LPC_GPIO_PORT, JOYSTICK_RIGHT_GPIO_PORT_NUM, JOYSTICK_RIGHT_GPIO_BIT_NUM) == 0) {
		ret |= JOY_RIGHT;
	}
	else if (Chip_GPIO_ReadPortBit(LPC_GPIO_PORT, JOYSTICK_PRESS_GPIO_PORT_NUM, JOYSTICK_PRESS_GPIO_BIT_NUM) == 0) {
		ret |= JOY_PRESS;
	}

	return ret;
}

/* Gets buttons status */
uint32_t Buttons_GetStatus(void)
{
	uint8_t ret = NO_BUTTON_PRESSED;

	if (Chip_GPIO_ReadPortBit(LPC_GPIO_PORT, BUTTONS_BUTTON1_GPIO_PORT_NUM, BUTTONS_BUTTON1_GPIO_BIT_NUM) == 0) {
		ret |= BUTTONS_BUTTON1;
	}
	return ret;
}

/* Initialize DAC interface for the board */
void Board_DAC_Init(LPC_DAC_T *pDAC)
{
	Chip_SCU_DAC_Analog_Config();
}

/* FIXME Should we remove this function? */
void Serial_CreateStream(void *Stream)
{
	// implement later
}

/**
 * @}
 */
