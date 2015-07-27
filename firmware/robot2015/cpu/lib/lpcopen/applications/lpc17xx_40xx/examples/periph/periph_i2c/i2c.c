/*
 * @brief I2C example
 * This example show how to use the I2C interface
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

#include <stdlib.h>
#include <string.h>
#include "board.h"

/** @defgroup EXAMPLES_PERIPH_17XX40XX_I2C LPC17xx/40xx I2C example
 * @ingroup EXAMPLES_PERIPH_17XX40XX
 * <b>Example description</b><br>
 * The I2C example shows how to use I2C interface in master and slave mode using
 * POLLING/INTERRUPT method.<br>
 *
 * To use the example, connect a serial cable to the board's RS232/UART port and
 * start a terminal program to monitor the port. The terminal program on the host
 * PC should be setup for 115K8N1.<br>
 *
 * For boards that does not have default UART, the user can enable debugging,
 * by defining ENABLE_DEBUG macro in sys_config.h and then connect FTDI UART cable
 * to the board to use the full functionality of the example. If no UART can be
 * connected see the paragraph below for using the example without uart<br>
 *
 *<br><i> If there is no UART connected to Xpresso board, and if ENABLE_DEBUG is not
 * defined then make connections as specified in the  "Special connection requirements"
 * section. When the board is reset or powered ON, the Master I2C will write to the
 * IO Expansion slave making the RED LED Glow, and the example will terminate</i><br><br>
 *
 * After a Main menu is displayed via UART, I2C0 and I2C1 are configured with
 * 100KHz speed.<br>
 *
 * User can access any I2C slave device (probe all devices by selecting option 3 from
 * main menu) connected to the bus using this example,
 * i.e, operations like WRITE/READ/WRITE&READ can be carried out by using any I2C
 * I2C0/I2C1 as master. The input from UART could be decimal, hexadecimal (prefixed
 * with 0x), or octal (prefixed with 0) and can be edited using backspace. User must
 * press enter key to complete the input.<br><br>
 * This example simulates two slave devices (default by I2C0), an EEPROM (0x5A) and
 * an IO Extension device (0x5B). The EEPROM simulation is just to show how the I2C
 * can be used as a device (it will not retain contents across power cycles). IO
 * extention device have 2 ports, any single byte read will read 8 bit data from IN
 * port (8 bit input reg0), any single byte write will write to OUT port (8 bit output
 * reg1). Multibyte reads will read (IN REG, OUT REG) repeatedly, multi byte writes will
 * write to OUT REG repeatedly. OUT REG represent LEDS in the board set/reset them will
 * turn LED0-LED7 (If present) to ON/OFF, IN REG represent key press (KEY0 to KEY7).
 *
 * To turn on LED1 and LED2, follow the steps given below <br>
 * 1. Select I2C1 as default device by selecting option 1 and 1<br>
 * 2. Select option 5 to write data to slave<br>
 * 3. Enter 0x5B as the slave device address<br>
 * 4. Enter 1 as the number of bytes to write<br>
 * 6. Enter 6 as the data [Turn ON LED1 and LED2]<br>
 *
 * The EEPROM slave (simulation) device will work as any standard I2C EEPROM. To write
 * data 0xDE, 0xAD, 0xBE, 0xEF to address 0x10, 0x11, 0x12 & 0x13 respectively and to
 * read back the full content of the EEPROM follow the steps
 * 1.  Select I2C1 as default device by choosing option 1 and give input 1<br>
 * 2.  Select option 5 to write data to slave<br>
 * 3.  Enter 0x5A as the slave device address<br>
 * 4.  Enter 5 as the number of bytes to write<br>
 * 5.  Enter 0x10 as the address to write<br>
 * <blockquote> First byte written to device is always considered as the address
 * from/to which read/write to be performed, the EEPROM uses 64 byte memory hence
 * BIT0 to BIT5 will have address A0-A5 and BIT6 & BIT7 are don't cares. For every
 * read/write the address will auto increment upon reaching the end of the device
 * the address will automatically roll back to location 0.</blockquote>
 * 6.  Enter data 0xDE,0xAD,0xBE,0xEF one by one<br>
 * 7.  Select option 6 to write/read the data from EEPROM<br>
 * 8.  Enter 0x5A as the slave address<br>
 * 9.  Enter 64 as the number of bytes to read<br>
 * 10. Enter 1 as the number of bytes to write<br>
 * 11. Enter 0 as the address from which read must start<br>
 * 12. Full content of EEPROM will be displayed on UART terminal with address
 * 0x10 to 0x13 showing the data written in previous operation.
 *
 * <b>Special connection requirements</b><br>
 *  - EA-DEVKIT 1788/4088<br>
 *    - I2C1_SDA (J5 pin 45) connected to I2C0_SDA (J5 pin 25) <br>
 *    - I2C1_SCL (J5 pin 54) connected to I2C0_SCL (J5 pin 26) <br>
 *    - Jumper @a J5 is the 64 pin expansion connecter close to 200pin OEM board connector <br>
 *    - Make sure double check the connected pins as there is a great possibility of connecting wrong pins<br>
 *  - LPCXpresso 1769 (using UART3)<br>
 *    - P0.19(SDA1) PAD17 connected to P0.27(SDA0) J6-PIN25<br>
 *    - P0.20(SCL1) PAD18 connected to P0.28(SCL0) J6-PIN26<br>
 *
 * <b>Build procedures:</b><br>
 * @ref LPCOPEN_17XX40XX_BUILDPROCS_KEIL<br>
 * @ref LPCOPEN_17XX40XX_BUILDPROCS_IAR<br>
 * @ref LPCOPEN_17XX40XX_BUILDPROCS_XPRESSO<br>
 *
 * <b>Supported boards and board setup:</b><br>
 * @ref LPCOPEN_17XX40XX_BOARD_EA1788<br>
 * @ref LPCOPEN_17XX40XX_BOARD_EA4088<br>
 * @ref LPCOPEN_17XX40XX_BOARD_XPRESSO_1769<br>
 *
 * <b>Submitting LPCOpen issues:</b><br>
 * @ref LPCOPEN_COMMUNITY
 * @{
 */

/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/
#define DEFAULT_I2C          I2C0

#define I2C_EEPROM_BUS       DEFAULT_I2C
#define I2C_IOX_BUS          DEFAULT_I2C

#define SPEED_100KHZ         100000
#define SPEED_400KHZ         400000

#ifdef DEBUG_ENABLE
static const char menu[] =
"**************** I2C Demo Menu ****************\r\n"
"\t0: Exit Demo\r\n"
"\t1: Select I2C peripheral [\033[1;32mI2C%d\033[0;37m]\r\n"
"\t2: Toggle mode POLLING/INTERRUPT [\033[1;32m%s\033[0;37m]\r\n"
"\t3: Probe for Slave devices\r\n"
"\t4: Read slave data\r\n"
"\t5: Write slave data\r\n"
"\t6: Write/Read slave data\r\n";
#endif

static int mode_poll;   /* Poll/Interrupt mode flag */
static I2C_ID_T i2cDev = DEFAULT_I2C; /* Currently active I2C device */

/* EEPROM SLAVE data */
#define I2C_SLAVE_EEPROM_SIZE       64
#define I2C_SLAVE_EEPROM_ADDR       0x5A
#define I2C_SLAVE_IOX_ADDR          0x5B

/* Xfer structure for slave operations */
static I2C_XFER_T seep_xfer;
static I2C_XFER_T iox_xfer;

/* Data area for slave operations */
static uint8_t seep_data[I2C_SLAVE_EEPROM_SIZE + 1];
static uint8_t buffer[2][256];
static uint8_t iox_data[2]; /* PORT0 input port, PORT1 Output port */

/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/

/*****************************************************************************
 * Private functions
 ****************************************************************************/

/* State machine handler for I2C0 and I2C1 */
static void i2c_state_handling(I2C_ID_T id)
{
	if (Chip_I2C_IsMasterActive(id)) {
		Chip_I2C_MasterStateHandler(id);
	} else {
		Chip_I2C_SlaveStateHandler(id);
	}
}

/* Print data to console */
static void con_print_data(const uint8_t *dat, int sz)
{
	int i;
	if (!sz) return;
	for (i = 0; i < sz; i++) {
		if (!(i & 0xF)) DEBUGOUT("\r\n%02X: ", i);
		DEBUGOUT(" %02X", dat[i]);
	}
	DEBUGOUT("\r\n");
}

/* Get an integer input from UART */
static int con_get_input(const char *str)
{
#ifdef DEBUG_ENABLE
	int input_valid = 0;
	int x;
	char ch[16], *ptr;
	int i = 0;

	while (!input_valid) {
		DEBUGOUT("%s", str);
		while(1) {
			/* Setting poll mode for slave is a very bad idea, it works nevertheless */
			if((mode_poll & (1 << i2cDev)) && Chip_I2C_IsStateChanged(i2cDev)) {
				Chip_I2C_SlaveStateHandler(i2cDev);
			}

			x = DEBUGIN();
			if (x == EOF) continue;
			if (i >= sizeof(ch) - 2) break;
			if ((x == '\r' || x == '\n') && i) {
				DEBUGOUT("\r\n");
				break;
			}
			if (x == '\b') {
				if (i) {
					DEBUGOUT("\033[1D \033[1D");
					i --;
				}
				continue;
			}
			DEBUGOUT("%c", x);
			ch[i++] = x;
		};
		ch[i] = 0;
		i = strtol(ch, &ptr, 0);
		if (*ptr) {
			i = 0;
			DEBUGOUT("Invalid input. Retry!\r\n");
			continue;
		}
		input_valid = 1;
	}
	return i;
#else
	static int sind = 0;
	static uint8_t val[] = {1, 1, 5, I2C_SLAVE_IOX_ADDR, 1, 1, 0};
	while(!val[sind]);

	return val[sind++];
#endif
}

static void i2c_rw_input(I2C_XFER_T *xfer, int ops)
{
	int tmp, i;

	tmp = con_get_input("Enter 7-Bit Slave address : ");
	tmp &= 0xFF;
	xfer->slaveAddr = tmp;
	xfer->rxBuff = 0;
	xfer->txBuff = 0;
	xfer->txSz = 0;
	xfer->rxSz = 0;

	if (ops & 1) {
		tmp = con_get_input("Enter number of bytes to read : ");
		tmp &= 0xFF;
		xfer->rxSz = tmp;
		xfer->rxBuff = buffer[1];
	}

	if (ops & 2) {
		tmp = con_get_input("Enter number of bytes to write : ");
		tmp &= 0xFF;
		for (i = 0; i < tmp; i++) {
			DEBUGOUT("%d:", i + 1);
			buffer[0][i] = con_get_input("Enter Data: ");
		}
		xfer->txSz = tmp;
		xfer->txBuff = buffer[0];
	}
}

/* Set I2C mode to polling/interrupt */
static void i2c_set_mode(I2C_ID_T id, int polling)
{
	if(!polling) {
		mode_poll &= ~(1 << id);
		Chip_I2C_SetMasterEventHandler(id, Chip_I2C_EventHandler);
		NVIC_EnableIRQ(id == I2C0 ? I2C0_IRQn : I2C1_IRQn);
	} else {
		mode_poll |= 1 << id;
		NVIC_DisableIRQ(id == I2C0 ? I2C0_IRQn : I2C1_IRQn);
		Chip_I2C_SetMasterEventHandler(id, Chip_I2C_EventHandlerPolling);
	}
}

/* Initialize the I2C bus */
static void i2c_app_init(I2C_ID_T id, int speed)
{
	Board_I2C_Init(id);

	/* Initialize I2C */
	Chip_I2C_Init(id);
	Chip_I2C_SetClockRate(id, speed);

	/* Set default mode to interrupt */
	i2c_set_mode(id, 0);
}

/* Function that probes all available slaves connected to an I2C bus */
static void i2c_probe_slaves(I2C_ID_T i2c)
{
	int i;
	uint8_t ch[2];

	DEBUGOUT("Probing available I2C devices...\r\n");
	DEBUGOUT("\r\n     00 01 02 03 04 05 06 07 08 09 0A 0B 0C 0D 0E 0F");
	DEBUGOUT("\r\n====================================================");
	for (i = 0; i <= 0x7F; i++) {
		if (!(i & 0x0F)) DEBUGOUT("\r\n%02X  ", i >> 4);
		if (i <= 7 || i > 0x78) {
			DEBUGOUT("   ");
			continue;
		}
		/* Address 0x48 points to LM75AIM device which needs 2 bytes be read */
		if(Chip_I2C_MasterRead(i2c, i, ch, 1 + (i == 0x48)) > 0)
			DEBUGOUT(" %02X", i);
		else
			DEBUGOUT(" --");
	}
	DEBUGOUT("\r\n");
}

static int i2c_menu(void)
{
	DEBUGOUT(menu, i2cDev, (mode_poll & (1 << i2cDev)) ? "POLLING" : "INTERRUPT");
	return con_get_input("\r\nSelect an option [0 - 6] :");
}

/* Update the EEPROM state */
static void i2c_eeprom_update_state(I2C_XFER_T *xfer, uint8_t *buff, int sz)
{
	xfer->txBuff = xfer->rxBuff = &buff[buff[0]+1];
	xfer->rxSz = xfer->txSz = sz - buff[0] + 1;
}

/* Slave event handler for simulated EEPROM */
static void i2c_eeprom_events(I2C_ID_T id, I2C_EVENT_T event)
{
	static int is_addr = 1;
	switch(event) {
		case I2C_EVENT_DONE:
			is_addr = 1;
			i2c_eeprom_update_state(&seep_xfer, seep_data, I2C_SLAVE_EEPROM_SIZE);
			seep_xfer.rxBuff = seep_data;
			seep_xfer.rxSz ++;
			break;

		case I2C_EVENT_SLAVE_RX:
			if (is_addr) {
				is_addr = 0;
				seep_data[0] &= (I2C_SLAVE_EEPROM_SIZE - 1); /* Correct addr if required */
				i2c_eeprom_update_state(&seep_xfer, seep_data, I2C_SLAVE_EEPROM_SIZE);
				break;
			}

			seep_data[0] ++;
			seep_data[0] &= (I2C_SLAVE_EEPROM_SIZE - 1);
			if (seep_xfer.rxSz == 1)
				i2c_eeprom_update_state(&seep_xfer, seep_data, I2C_SLAVE_EEPROM_SIZE);
			break;

		case I2C_EVENT_SLAVE_TX:
			seep_data[0] ++;
			seep_data[0] &= (I2C_SLAVE_EEPROM_SIZE - 1);
			if (seep_xfer.txSz == 1)
				i2c_eeprom_update_state(&seep_xfer, seep_data, I2C_SLAVE_EEPROM_SIZE);
			break;
	}
}

/* Simulate an I2C EEPROM slave */
static void i2c_eeprom_init(I2C_ID_T id)
{
	memset(&seep_data[1], 0xFF, I2C_SLAVE_EEPROM_SIZE);
	seep_xfer.slaveAddr = (I2C_SLAVE_EEPROM_ADDR << 1);
	seep_xfer.txBuff = &seep_data[1];
	seep_xfer.rxBuff = seep_data;
	seep_xfer.txSz = seep_xfer.rxSz = sizeof(seep_data);
	Chip_I2C_SlaveSetup(id, I2C_SLAVE_0, &seep_xfer, i2c_eeprom_events, 0);
}
/*-------- IO Expansion slave device implementation ----------*/
/* Update IN/OUT port states to real devices */
void i2c_iox_update_regs(int ops)
{
	if (ops & 1) { /* update out port */
		Board_LED_Set(0, iox_data[1] & 1);
		Board_LED_Set(1, iox_data[1] & 2);
		Board_LED_Set(2, iox_data[1] & 4);
		Board_LED_Set(3, iox_data[1] & 8);
	}

	if (ops & 2) { /* update in port */
		iox_data[0] = (uint8_t) Buttons_GetStatus();
	}
}

/* Slave event handler for simulated EEPROM */
static void i2c_iox_events(I2C_ID_T id, I2C_EVENT_T event)
{
	switch(event) {
		case I2C_EVENT_DONE:
			iox_xfer.rxBuff = &iox_data[1];
			iox_xfer.rxSz = sizeof(iox_data);
			iox_xfer.txBuff = (const uint8_t *)iox_data;
			iox_xfer.txSz = sizeof(iox_data) + 1;
			break;

		case I2C_EVENT_SLAVE_RX:
			iox_xfer.rxBuff = &iox_data[1];
			iox_xfer.rxSz = sizeof(iox_data);
			i2c_iox_update_regs(1);
			break;

		case I2C_EVENT_SLAVE_TX:
			if(iox_xfer.txSz == 1) {
				iox_xfer.txBuff = (const uint8_t *)iox_data[0];
				iox_xfer.txSz = sizeof(iox_data) + 1;
			}
			break;
	}
}

/* Simulate an IO Expansion slave device */
static void i2c_iox_init(I2C_ID_T id)
{
	Board_Buttons_Init();
	iox_xfer.slaveAddr = (I2C_SLAVE_IOX_ADDR << 1);
	i2c_iox_events(id, I2C_EVENT_DONE);
	Chip_I2C_SlaveSetup(id, I2C_SLAVE_1, &iox_xfer, i2c_iox_events, 0);
	i2c_iox_update_regs(3);
	/* Setup SysTick timer to get the button status updated at regular intervals */
	SysTick_Config(Chip_Clock_GetSystemClockRate() / 50);
}

/*-------------------- End of IO Expansion slave device ----------------------*/

/*****************************************************************************
 * Public functions
 ****************************************************************************/
/**
 * @brief	SysTick Interrupt Handler
 * @return	Nothing
 * @note	Systick interrupt handler updates the button status
 */
void SysTick_Handler(void)
{
	i2c_iox_update_regs(2);
}

/**
 * @brief	I2C Interrupt Handler
 * @return	None
 */
void I2C1_IRQHandler(void)
{
	i2c_state_handling(I2C1);
}

/**
 * @brief	I2C0 Interrupt handler
 * @return	None
 */
void I2C0_IRQHandler(void)
{
	i2c_state_handling(I2C0);
}

/**
 * @brief	Main program body
 * @return	int
 */
int main(void)
{
	int tmp;
	int xflag = 0;
	static I2C_XFER_T xfer;

	Board_Init();
	i2c_app_init(I2C0, SPEED_100KHZ);
	i2c_app_init(I2C1, SPEED_100KHZ);

	/* Simulate an EEPROM slave in I2C0 */
	i2c_eeprom_init(I2C_EEPROM_BUS);

	/* Simuldate an IO Expansion slave in I2C0 */
	i2c_iox_init(I2C_IOX_BUS);

	while (!xflag) {
		switch(i2c_menu()) {
			case 0:
				xflag = 1;
				DEBUGOUT("End of I2C Demo! Bye!\r\n");
				break;

			case 1:
				tmp = con_get_input("Select I2C device [0 or 1] : ");
				DEBUGOUT("\r\n");
				if ((I2C_ID_T) tmp == I2C0) {
					if (i2cDev == I2C0) break;
					i2c_set_mode(I2C0, 0);
					i2cDev = I2C0;
				}
				else if((I2C_ID_T) tmp == I2C1) {
					if (i2cDev == I2C1) break;
					i2c_set_mode(I2C1, 0);
					i2cDev = I2C1;
				}
				else
					DEBUGOUT("Invalid I2C Device [Must be 0 or 1]\r\n");
				break;

			case 2:
				i2c_set_mode(i2cDev, !(mode_poll & (1 << i2cDev)));
				break;

			case 3:
				i2c_probe_slaves(i2cDev);
				break;

			case 4:
				i2c_rw_input(&xfer, 1);
				tmp = Chip_I2C_MasterRead(i2cDev, xfer.slaveAddr, xfer.rxBuff, xfer.rxSz);
				DEBUGOUT("Read %d bytes of data from slave 0x%02X.\r\n", tmp, xfer.slaveAddr);
				con_print_data(buffer[1], tmp);
				break;

			case 5:
				i2c_rw_input(&xfer, 2);
				if (xfer.txSz == 0) break;
				tmp = Chip_I2C_MasterSend(i2cDev, xfer.slaveAddr, xfer.txBuff, xfer.txSz);
				DEBUGOUT("Written %d bytes of data to slave 0x%02X.\r\n", tmp, xfer.slaveAddr);
				break;

			case 6:
				i2c_rw_input(&xfer, 3);
				tmp = xfer.rxSz;
				if (!tmp && !xfer.txSz) break;
				Chip_I2C_MasterTransfer(i2cDev, &xfer);
				DEBUGOUT("Master transfer : %s\r\n",
					xfer.status == I2C_STATUS_DONE ? "SUCCESS" : "FAILURE");
				DEBUGOUT("Received %d bytes from slave 0x%02X\r\n", tmp - xfer.rxSz, xfer.slaveAddr);
				con_print_data(buffer[1], tmp - xfer.rxSz);
				break;

			default:
				DEBUGOUT("Input Invalid! Try Again.\r\n");
		}
	}
	Chip_I2C_DeInit(I2C0);
	Chip_I2C_DeInit(I2C1);

	return 0;
}

/**
 * @}
 */
