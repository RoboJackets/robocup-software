/*
 * @brief	Memory ethernet example
 *			This example shows how to do a (more complex) memory test for
 *			verifying DRAM operation. The test performs walking 0 and 1,
 *			address and inverse address, and pattern tests on DRAM.
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
#include "mem_tests.h"
#include <stdio.h>

/** @defgroup EXAMPLES_PERIPH_17XX40XX_EMEMT LPC17xx/40xx Memory tests
 * @ingroup EXAMPLES_PERIPH_17XX40XX
 * <b>Example description</b><br>
 * This example runs a few memory tests on external DRAM memory. The tests include
 * walking 0 and 1, address and inverse address, and pattern tests.<br>
 *
 * These tests are meant to be run via a debugger inside IRAM and will not run
 * standalone.<br>
 *
 * <b>Special connection requirements</b><br>
 * There are no special connection requirements for this example.<br>
 *
 * <b>Build procedures:</b><br>
 * @ref LPCOPEN_17XX40XX_BUILDPROCS_KEIL<br>
 * @ref LPCOPEN_17XX40XX_BUILDPROCS_IAR<br>
 * @ref LPCOPEN_17XX40XX_BUILDPROCS_XPRESSO<br>
 *
 * <b>Supported boards and board setup:</b><br>
 * @ref LPCOPEN_17XX40XX_BOARD_EA1788<br>
 * @ref LPCOPEN_17XX40XX_BOARD_EA4088<br>
 *
 * <b>Submitting LPCOpen issues:</b><br>
 * @ref LPCOPEN_COMMUNITY
 * @{
 */

/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/

#define DRAM_BASE_ADDRESS (uint32_t *) EMC_ADDRESS_DYCS0
#define DRAM_SIZE (8 * 1024 * 1024)

/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/

/*****************************************************************************
 * Private functions
 ****************************************************************************/

/*****************************************************************************
 * Public functions
 ****************************************************************************/

/**
 * @brief	Main routine for example_memtest
 * @return	Nothing
 */
int main(void)
{
	MEM_TEST_SETUP_T memSetup;

	Board_Init();

	/* Walking 0 test */
	memSetup.start_addr = DRAM_BASE_ADDRESS;
	memSetup.bytes = DRAM_SIZE;
	if (mem_test_walking0(&memSetup)) {
		DEBUGSTR("Walking 0 memory test passed\r\n");
	}
	else {
		DEBUGOUT("Walking 0 memory test failed at address %p\r\n", memSetup.fail_addr);
		DEBUGOUT(" Expected %08x, actual %08x\r\n", memSetup.ex_val, memSetup.is_val);
	}

	/* Walking 1 test */
	memSetup.start_addr = DRAM_BASE_ADDRESS;
	memSetup.bytes = DRAM_SIZE;
	if (mem_test_walking1(&memSetup)) {
		DEBUGSTR("Walking 1 memory test passed\r\n");
	}
	else {
		DEBUGOUT("Walking 1 memory test failed at address %p\r\n", memSetup.fail_addr);
		DEBUGOUT(" Expected %08x, actual %08x\r\n", memSetup.ex_val, memSetup.is_val);
	}

	/* Address test */
	memSetup.start_addr = DRAM_BASE_ADDRESS;
	memSetup.bytes = DRAM_SIZE;
	if (mem_test_address(&memSetup)) {
		DEBUGSTR("Address test passed\r\n");
	}
	else {
		DEBUGOUT("Address test failed at address %p\r\n", memSetup.fail_addr);
		DEBUGOUT(" Expected %08x, actual %08x\r\n", memSetup.ex_val, memSetup.is_val);
	}

	/* Inverse address test */
	memSetup.start_addr = DRAM_BASE_ADDRESS;
	memSetup.bytes = DRAM_SIZE;
	if (mem_test_invaddress(&memSetup)) {
		DEBUGSTR("Inverse address test passed\r\n");
	}
	else {
		DEBUGOUT("Inverse address test failed at address %p\r\n", memSetup.fail_addr);
		DEBUGOUT(" Expected %08x, actual %08x\r\n", memSetup.ex_val, memSetup.is_val);
	}

	/* Pattern test */
	memSetup.start_addr = DRAM_BASE_ADDRESS;
	memSetup.bytes = DRAM_SIZE;
	if (mem_test_pattern(&memSetup)) {
		DEBUGSTR("Pattern (0x55/0xAA) test passed\r\n");
	}
	else {
		DEBUGOUT("Pattern (0x55/0xAA) test failed at address %p\r\n", memSetup.fail_addr);
		DEBUGOUT(" Expected %08x, actual %08x\r\n", memSetup.ex_val, memSetup.is_val);
	}

	/* Seeded pattern test */
	memSetup.start_addr = DRAM_BASE_ADDRESS;
	memSetup.bytes = DRAM_SIZE;
	if (mem_test_pattern_seed(&memSetup, 0x12345678, 0x50005)) {
		DEBUGSTR("Seeded pattern test passed\r\n");
	}
	else {
		DEBUGOUT("Seeded pattern test failed at address %p\r\n", memSetup.fail_addr);
		DEBUGOUT(" Expected %08x, actual %08x\r\n", memSetup.ex_val, memSetup.is_val);
	}

	return 0;
}

/**
 * @}
 */
