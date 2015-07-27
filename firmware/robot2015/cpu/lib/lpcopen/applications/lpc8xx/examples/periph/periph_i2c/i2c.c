/*
 * @brief I2C example
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

/** @defgroup EXAMPLES_PERIPH_8XX_I2C LPC8xx I2C example using I2C ROM API functions
 * @ingroup EXAMPLES_PERIPH_8XX
 * <b>Example description</b><br>
 * This example shows how to configure I2C as master or slave device using ROM-based APIs.<br>
 *
 * This demo supports both 7-bit and 10-bit slave addressing. After setting up successfully,
 * all transmit/receive functions will be called step by step.<br>
 * If error occurs, this example will hang up. When all steps run successfully, the demo will 
 * end and blue LED will be shown.<br>
 *
 * <b>Special connection requirements</b><br>
 * User needs 2 boards connected through P0.10 (SDA), P0.11 (SCL) and a common GND (need two
 * pull-up resistors for I2C bus because P0.10 and P0.11 are special I2C open-drain pins).<br>
 *
 * <b>Build procedures:</b><br>
 * @ref LPCOPEN_8XX_BUILDPROCS_IAR<br>
 * @ref LPCOPEN_8XX_BUILDPROCS_KEIL<br>
 * @ref LPCOPEN_8XX_BUILDPROCS_XPRESSO<br>
 *
 * <b>Supported boards and board setup:</b><br>
 * @ref LPCOPEN_8XX_BOARD_XPRESSO_812<br>
 *
 * <b>Submitting LPCOpen issues:</b><br>
 * @ref LPCOPEN_COMMUNITY
 * @{
 */

/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/

/** I2C mode: 1 - Master Mode, 0 - Slave Mode */
#define I2C_MASTER_MODE         1
/** 7-bit slave address */
#define I2C_SLAVE_ADDR_7BIT     (0x90)
/** 10-bit slave address */
#define I2C_SLAVE_ADDR_10BIT    (0x2CA)
/** Memory size for I2C ROM */
#define I2C_ROM_MEM_SIZE        (0x100UL)
/** 1Mbps I2C bit-rate */
#define I2C_BITRATE             (1000000)
/** Max buffer length */
#define BUFFER_SIZE             (0xFF)

static I2C_HANDLE_T     *i2c_handle;
static uint32_t         i2cmem[I2C_ROM_MEM_SIZE];
volatile uint32_t       error_code;
I2C_PARAM_T             param;
I2C_RESULT_T            result;
volatile bool   isTxCompleted = false;
volatile bool   isRxCompleted = false;
volatile bool   isTxRxCompleted = false;

uint8_t Buffer[BUFFER_SIZE];
uint8_t TmpBuffer[BUFFER_SIZE];

/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/

/*****************************************************************************
 * Private functions
 ****************************************************************************/

/* Initialize buffer */
static void Buffer_Init(uint8_t *buffer, uint32_t buf_len, uint8_t type)
{
	uint8_t i;

	if (type) {
		for (i = 0; i < buf_len; i++) {
			buffer[i] = i;
		}
	}
	else {
		for (i = 0; i < buf_len; i++) {
			buffer[i] = 0;
		}
	}
}

/* If error occurs, turn LED 0 (red LED) on and wait forever */
static void spin_on_error(void)
{
	while (1) {
		/* Show the red LED continuously due to error */
		Board_LED_Toggle(0);
	}
}

/* Compare two buffers from position StartPos to StopPos */
static void Buffer_Compare(uint8_t *buffer1, uint8_t *buffer2, uint32_t StartPos,  uint32_t StopPos)
{
	uint32_t i = 0;
	for (i = StartPos; i < StopPos; i++) {
		if (buffer1[i] != buffer2[i]) {
			/* buffer1 differs from buffer2 and red LED is shown due to error */
			spin_on_error();
		}
	}
}

/* I2C Receive Callback function */
static void App_I2C_Callback_Receive(uint32_t err_code, uint32_t n)
{
	isRxCompleted = true;
	error_code = err_code;
}

/* I2C Transmit Callback function */
static void App_I2C_Callback_Transmit(uint32_t err_code, uint32_t n)
{
	isTxCompleted = true;
	error_code = err_code;
}
#if I2C_MASTER_MODE 
/* I2C Transmit and Receive Callback function */
static void App_I2C_Callback_MasterTxRx(uint32_t err_code, uint32_t n)
{
	isTxRxCompleted = true;
	error_code = err_code;
}
#endif

#if I2C_MASTER_MODE
/* Master Polling Mode */
static void App_I2C_Master_Polling(void)
{
	/* Transmit TmpBuffer */
	/* Initialize buffer */
	Buffer_Init(TmpBuffer, BUFFER_SIZE, 1);
	TmpBuffer[0] = I2C_SLAVE_ADDR_7BIT;

	param.num_bytes_send    = BUFFER_SIZE;
	param.buffer_ptr_send   = &TmpBuffer[0];
	param.stop_flag         = 1;
	do {
		error_code = LPC_I2CD_API->i2c_master_transmit_poll(i2c_handle, &param, &result);
	} while (error_code);

	/* Receive Buffer and compare with TmpBuffer */
	/* Initialize buffer */
	Buffer_Init(Buffer, BUFFER_SIZE, 0);
	Buffer[0] = I2C_SLAVE_ADDR_7BIT | 0x01;

	param.num_bytes_recv    = BUFFER_SIZE;
	param.buffer_ptr_recv   = &Buffer[0];
	param.stop_flag         = 1;
	do {
		error_code = LPC_I2CD_API->i2c_master_receive_poll(i2c_handle, &param, &result);
	} while (error_code);

	/* Verify */
	Buffer_Compare(TmpBuffer, Buffer, 1, BUFFER_SIZE);

	/* Transmit TmpBuffer and receive Buffer, then compare them together */
	/* Initialize buffer */
	Buffer_Init(Buffer, BUFFER_SIZE, 0);
	Buffer[0] = I2C_SLAVE_ADDR_7BIT | 0x01;

	TmpBuffer[0] = I2C_SLAVE_ADDR_7BIT;
	TmpBuffer[1] = 0xBA;

	param.num_bytes_send    = BUFFER_SIZE;
	param.buffer_ptr_send   = &TmpBuffer[0];
	param.num_bytes_recv    = BUFFER_SIZE;
	param.buffer_ptr_recv   = &Buffer[0];
	param.stop_flag         = 1;

	do {
		error_code = LPC_I2CD_API->i2c_master_tx_rx_poll(i2c_handle, &param, &result);
	} while (error_code);

	/* Verify */
	Buffer_Compare(TmpBuffer, Buffer, 1, BUFFER_SIZE);
}

/* Master Interrupt Mode */
static void App_I2C_Master_Interrupt(void)
{
	/* Transmit TmpBuffer */
	/* Initialize buffer */
	Buffer_Init(TmpBuffer, BUFFER_SIZE, 1);
	TmpBuffer[0] = I2C_SLAVE_ADDR_7BIT;

	param.num_bytes_send    = BUFFER_SIZE;
	param.buffer_ptr_send   = &TmpBuffer[0];
	param.func_pt           = App_I2C_Callback_Transmit;
	param.stop_flag         = 1;

	NVIC_DisableIRQ(I2C_IRQn);
	NVIC_ClearPendingIRQ(I2C_IRQn);
	NVIC_EnableIRQ(I2C_IRQn);

	do {
		isTxCompleted = false;
		LPC_I2CD_API->i2c_master_transmit_intr(i2c_handle, &param, &result);
		while (!isTxCompleted) {}
	} while (error_code);

	/* Receive Buffer and compare with TmpBuffer */
	/* Initialize buffer */
	Buffer_Init(Buffer, BUFFER_SIZE, 0);
	Buffer[0] = I2C_SLAVE_ADDR_7BIT | 0x01;

	param.num_bytes_recv    = BUFFER_SIZE;
	param.buffer_ptr_recv   = &Buffer[0];
	param.func_pt           = App_I2C_Callback_Receive;
	param.stop_flag         = 1;

	do {
		isRxCompleted = false;
		LPC_I2CD_API->i2c_master_receive_intr(i2c_handle, &param, &result);
		while (!isRxCompleted) {}
	} while (error_code);

	/* Verify */
	Buffer_Compare(TmpBuffer, Buffer, 1, BUFFER_SIZE);

	/* Transmit TmpBuffer and receive Buffer, then compare them together */
	/* Initialize buffer */
	Buffer_Init(Buffer, BUFFER_SIZE, 0);
	Buffer[0] = I2C_SLAVE_ADDR_7BIT | 0x01;

	TmpBuffer[0] = I2C_SLAVE_ADDR_7BIT;
	TmpBuffer[1] = 0xBA;

	param.num_bytes_send    = BUFFER_SIZE;
	param.buffer_ptr_send   = &TmpBuffer[0];
	param.num_bytes_recv    = BUFFER_SIZE;
	param.buffer_ptr_recv   = &Buffer[0];
	param.func_pt           = App_I2C_Callback_MasterTxRx;
	param.stop_flag					= 1;

	do {
		isTxRxCompleted = false;
		LPC_I2CD_API->i2c_master_tx_rx_intr(i2c_handle, &param, &result);
		while (!isTxRxCompleted) {}
	} while (error_code);

	/* Verify */
	Buffer_Compare(TmpBuffer, Buffer, 1, BUFFER_SIZE);
}

/* Master Mode with 10-bit address */
static void App_I2C_Master_10bitAddressTest(void)
{
	/* Transmit Buffer to slave with 10-bit addressing mode */
	/* Initialize buffer */
	Buffer_Init(&TmpBuffer[1], (BUFFER_SIZE - 1), 1);
	Buffer_Init(Buffer, BUFFER_SIZE, 0);

	TmpBuffer[0] = ((I2C_SLAVE_ADDR_10BIT >> 7) & 0x06) | 0xF0;	/* 4 MSBs of slave address in first byte of transmit buffer */
	TmpBuffer[1] = I2C_SLAVE_ADDR_10BIT & 0x0FF;

	param.num_bytes_send    = BUFFER_SIZE;
	param.buffer_ptr_send   = &TmpBuffer[0];
	param.func_pt           = App_I2C_Callback_Transmit;

	NVIC_DisableIRQ(I2C_IRQn);
	NVIC_ClearPendingIRQ(I2C_IRQn);
	NVIC_EnableIRQ(I2C_IRQn);

	do {
		isTxCompleted = false;
		LPC_I2CD_API->i2c_master_transmit_intr(i2c_handle, &param, &result);
		while (!isTxCompleted) {}
	} while (error_code);

	/* Receive buffer from slave with 10-bit addressing mode */
	Buffer[0] = ((I2C_SLAVE_ADDR_10BIT >> 7) & 0x06) | 0xF0;

	param.num_bytes_send    = 2;
	param.buffer_ptr_send   = &TmpBuffer[0];
	param.num_bytes_recv    = BUFFER_SIZE;
	param.buffer_ptr_recv   = &Buffer[0];
	param.func_pt           = App_I2C_Callback_MasterTxRx;

	NVIC_DisableIRQ(I2C_IRQn);
	NVIC_ClearPendingIRQ(I2C_IRQn);
	NVIC_EnableIRQ(I2C_IRQn);

	do {
		isTxRxCompleted = false;
		LPC_I2CD_API->i2c_master_tx_rx_intr(i2c_handle, &param, &result);
		while (!isTxRxCompleted) {}
	} while (error_code);

	/* Verify */
	Buffer_Compare(TmpBuffer, Buffer, 0, BUFFER_SIZE);
}

#else
/* Slave Polling Mode */
static void App_I2C_Slave_Polling(void)
{
	/* Receive an array of bytes from master */
	/* Initialize buffer */
	Buffer_Init(Buffer, BUFFER_SIZE, 0);
	Buffer_Init(TmpBuffer, BUFFER_SIZE, 1);

	param.num_bytes_recv    = BUFFER_SIZE;
	param.buffer_ptr_recv   = &Buffer[0];
	param.stop_flag         = 1;
	do {
		error_code = LPC_I2CD_API->i2c_slave_receive_poll(i2c_handle, &param, &result);
	} while (error_code);

	/* Verify */
	Buffer_Compare(TmpBuffer, Buffer, 1, BUFFER_SIZE);

	/* Transmit the buffer received */
	param.num_bytes_send    = BUFFER_SIZE - 1;
	param.buffer_ptr_send   = &Buffer[1];
	param.stop_flag         = 1;
	do {
		error_code = LPC_I2CD_API->i2c_slave_transmit_poll(i2c_handle, &param, &result);
	} while (error_code);

	/* Receive an array of byte from master; if Buffer[1] is equal to 0xBA, send the received buffer to master */
	/* Initialize buffer */
	Buffer_Init(Buffer, BUFFER_SIZE, 0);

	param.num_bytes_recv    = BUFFER_SIZE;
	param.buffer_ptr_recv   = &Buffer[0];
	param.stop_flag         = 1;
	do {
		error_code = LPC_I2CD_API->i2c_slave_receive_poll(i2c_handle, &param, &result);
	} while (error_code);

	if (Buffer[1] == 0xBA) {
		param.num_bytes_send    = BUFFER_SIZE - 1;
		param.buffer_ptr_send   = &Buffer[1];
		param.stop_flag         = 1;
		do {
			error_code = LPC_I2CD_API->i2c_slave_transmit_poll(i2c_handle, &param, &result);
		} while (error_code);
	}
}

/* Slave Interrupt Mode */
static void App_I2C_Slave_Interrupt(void)
{
	/* Receive an array of bytes from master */
	/* Initialize buffer */
	Buffer_Init(Buffer, BUFFER_SIZE, 0);
	Buffer_Init(TmpBuffer, BUFFER_SIZE, 1);

	param.num_bytes_recv    = BUFFER_SIZE;
	param.buffer_ptr_recv   = &Buffer[0];
	param.func_pt           = App_I2C_Callback_Receive;
	param.stop_flag         = 1;

	NVIC_DisableIRQ(I2C_IRQn);
	NVIC_ClearPendingIRQ(I2C_IRQn);
	NVIC_EnableIRQ(I2C_IRQn);

	do {
		isRxCompleted = false;
		LPC_I2CD_API->i2c_slave_receive_intr(i2c_handle, &param, &result);
		while (!isRxCompleted) {}
	} while (error_code);

	/* Verify */
	Buffer_Compare(TmpBuffer, Buffer, 1, BUFFER_SIZE);

	/* Transmit the buffer received */
	param.num_bytes_send    = BUFFER_SIZE - 1;
	param.buffer_ptr_send   = &TmpBuffer[1];
	param.func_pt           = App_I2C_Callback_Transmit;
	param.stop_flag         = 1;

	do {
		isTxCompleted = false;
		LPC_I2CD_API->i2c_slave_transmit_intr(i2c_handle, &param, &result);
		while (!isTxCompleted) {}
	} while (error_code);

	/* Receive an array of byte from master; if Buffer[1] is equal to 0xBA, send the received buffer to master */
	/* Initialize buffer */
	Buffer_Init(Buffer, BUFFER_SIZE, 0);

	param.num_bytes_recv    = BUFFER_SIZE;
	param.buffer_ptr_recv   = &Buffer[0];
	param.func_pt           = App_I2C_Callback_Receive;
	param.stop_flag         = 1;

	do {
		isRxCompleted = false;
		LPC_I2CD_API->i2c_slave_receive_intr(i2c_handle, &param, &result);
		while (!isRxCompleted) {}
	} while (error_code);

	if (Buffer[1] == 0xBA) {
		param.num_bytes_send    = BUFFER_SIZE - 1;
		param.buffer_ptr_send   = &Buffer[1];
		param.func_pt           = App_I2C_Callback_Transmit;
		param.stop_flag         = 1;

		do {
			isTxCompleted = false;
			LPC_I2CD_API->i2c_slave_transmit_intr(i2c_handle, &param, &result);
			while (!isTxCompleted) {}
		} while (error_code);
	}
}

/* Slave Mode with 10-bit address */
static void App_I2C_Slave_10bitAddressTest(void)
{
	/* Receive buffer from master */
	/* Initialize buffer */
	Buffer_Init(Buffer, BUFFER_SIZE, 0);

	param.num_bytes_recv    = BUFFER_SIZE;
	param.buffer_ptr_recv   = &Buffer[0];
	param.func_pt           = App_I2C_Callback_Receive;
	param.stop_flag         = 1;

	NVIC_DisableIRQ(I2C_IRQn);
	NVIC_ClearPendingIRQ(I2C_IRQn);
	NVIC_EnableIRQ(I2C_IRQn);

	do {
		isRxCompleted = false;
		LPC_I2CD_API->i2c_slave_receive_intr(i2c_handle, &param, &result);
		while (!isRxCompleted) {}
	} while (error_code);

	/* Send received buffer back to master */
	param.num_bytes_send    = BUFFER_SIZE - 1; /* Only send data + 1 byte slave address(only LSB 8bits) */
	param.buffer_ptr_send   = &Buffer[1];
	param.func_pt           = App_I2C_Callback_Transmit;
	param.stop_flag         = 1;

	do {
		isTxCompleted = false;
		LPC_I2CD_API->i2c_slave_transmit_intr(i2c_handle, &param, &result);
		while (!isTxCompleted) {}
	} while (error_code);
}

#endif

/*****************************************************************************
 * Public functions
 ****************************************************************************/

/**
 * @brief	I2C Interrupt Service Routine
 * @return	Nothing
 */
void I2C_IRQHandler(void)
{
//	Board_LED_Toggle(1);
	LPC_I2CD_API->i2c_isr_handler(i2c_handle);
}

/**
 * @brief	Set up hardware for lpc8xx I2C example
 * @return	Nothing
 */
void HW_Setup(void)
{
	volatile uint32_t mem;

	/* Generic Initialization */
	Board_Init();
	Board_I2C_Init();
	Chip_I2C_Init();

	/* Get the I2C memory size needed */
	mem = LPC_I2CD_API->i2c_get_mem_size();

	/* Perform a sanity check on the storage allocation */
	if (I2C_ROM_MEM_SIZE < (mem / sizeof(uint32_t))) {
		spin_on_error();
	}

	/* Setup the I2C */
	i2c_handle = LPC_I2CD_API->i2c_setup(LPC_I2C_BASE, i2cmem);

	/* Check the API return value for a valid handle */
	if (i2c_handle != NULL) {
		/* initialize the I2C with the configuration parameters */
		error_code = LPC_I2CD_API->i2c_set_bitrate(i2c_handle, Chip_Clock_GetSystemClockRate(), I2C_BITRATE);
		if (error_code) {
			spin_on_error();
		}
	}
}

/**
 * @brief	Main routine for I2C example.
 * @return	Function should not exit.
 */
int main(void) {
	HW_Setup();
	
	Board_LED_Set(0, false);
	Board_LED_Set(1, false);

#if I2C_MASTER_MODE
	App_I2C_Master_Polling();
	App_I2C_Master_Interrupt();
	App_I2C_Master_10bitAddressTest();

#else
	error_code =
		LPC_I2CD_API->i2c_set_slave_addr(i2c_handle,
										 ((I2C_SLAVE_ADDR_7BIT << 8) | (((I2C_SLAVE_ADDR_10BIT >> 7) & 0x06) | 0xF0)),
										 (0 | ((I2C_SLAVE_ADDR_10BIT & 0x0FF) << 8)  ));
	if (error_code) {
		spin_on_error();
	}

	App_I2C_Slave_Polling();
	App_I2C_Slave_Interrupt();
	App_I2C_Slave_10bitAddressTest();

#endif
	/* Show the blue LED if no errors occurs */
	Board_LED_Set(1, true);
	
	while (1) {
		__WFE();
	}
	return 0;
}

/**
 * @}
 */
