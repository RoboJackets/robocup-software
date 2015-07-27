// *****************************************************************************
//   +--+
//   | ++----+
//   +-++    |
//     |     |
//   +-+--+  |
//   | +--+--+
//   +----+    Copyright (c) 2012 Code Red Technologies Ltd.
//
// LPC8xx Microcontroller Startup code for use with Red Suite
//
// Version : 121107
//
// Software License Agreement
//
// The software is owned by Code Red Technologies and/or its suppliers, and is
// protected under applicable copyright laws.  All rights are reserved.  Any
// use in violation of the foregoing restrictions may subject the user to criminal
// sanctions under applicable laws, as well as to civil liability for the breach
// of the terms and conditions of this license.
//
// THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
// OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
// MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
// USE OF THIS SOFTWARE FOR COMMERCIAL DEVELOPMENT AND/OR EDUCATION IS SUBJECT
// TO A CURRENT END USER LICENSE AGREEMENT (COMMERCIAL OR EDUCATIONAL) WITH
// CODE RED TECHNOLOGIES LTD.
//
// *****************************************************************************
#if defined(__cplusplus)
#ifdef __REDLIB__
#error Redlib does not support C++
#else
// *****************************************************************************
//
// The entry point for the C++ library startup
//
// *****************************************************************************
extern "C" {
extern void __libc_init_array(void);

}
#endif
#endif

#define WEAK __attribute__ ((weak))
#define ALIAS(f) __attribute__ ((weak, alias(# f)))

// #if defined (__USE_CMSIS)
// #include "LPC8xx.h"
// #endif

// *****************************************************************************
#if defined(__cplusplus)
extern "C" {
#endif

// *****************************************************************************
//
// Forward declaration of the default handlers. These are aliased.
// When the application defines a handler (with the same name), this will
// automatically take precedence over these weak definitions
//
// *****************************************************************************
void ResetISR(void);
WEAK void NMI_Handler(void);
WEAK void HardFault_Handler(void);
WEAK void SVC_Handler(void);
WEAK void PendSV_Handler(void);
WEAK void SysTick_Handler(void);
WEAK void IntDefaultHandler(void);

// *****************************************************************************
//
// Forward declaration of the specific IRQ handlers. These are aliased
// to the IntDefaultHandler, which is a 'forever' loop. When the application
// defines a handler (with the same name), this will automatically take
// precedence over these weak definitions
//
// *****************************************************************************
void SPI0_IRQHandler(void) ALIAS(IntDefaultHandler);
void SPI1_IRQHandler(void) ALIAS(IntDefaultHandler);
void UART0_IRQHandler(void) ALIAS(IntDefaultHandler);
void UART1_IRQHandler(void) ALIAS(IntDefaultHandler);
void UART2_IRQHandler(void) ALIAS(IntDefaultHandler);
void I2C_IRQHandler(void) ALIAS(IntDefaultHandler);
void SCT_IRQHandler(void) ALIAS(IntDefaultHandler);
void MRT_IRQHandler(void) ALIAS(IntDefaultHandler);
void CMP_IRQHandler(void) ALIAS(IntDefaultHandler);
void WDT_IRQHandler(void) ALIAS(IntDefaultHandler);
void BOD_IRQHandler(void) ALIAS(IntDefaultHandler);
void FLASH_IRQHandler(void) ALIAS(IntDefaultHandler);
void WKT_IRQHandler(void) ALIAS(IntDefaultHandler);
void PININT0_IRQHandler(void) ALIAS(IntDefaultHandler);
void PININT1_IRQHandler(void) ALIAS(IntDefaultHandler);
void PININT2_IRQHandler(void) ALIAS(IntDefaultHandler);
void PININT3_IRQHandler(void) ALIAS(IntDefaultHandler);
void PININT4_IRQHandler(void) ALIAS(IntDefaultHandler);
void PININT5_IRQHandler(void) ALIAS(IntDefaultHandler);
void PININT6_IRQHandler(void) ALIAS(IntDefaultHandler);
void PININT7_IRQHandler(void) ALIAS(IntDefaultHandler);

// *****************************************************************************
//
// The entry point for the application.
// __main() is the entry point for Redlib based applications
// main() is the entry point for Newlib based applications
//
// *****************************************************************************
#if defined(__REDLIB__)
extern void __main(void);

#endif
extern int main(void);

// *****************************************************************************
//
// External declaration for the pointer to the stack top from the Linker Script
//
// *****************************************************************************
extern void _vStackTop(void);

// *****************************************************************************
#if defined(__cplusplus)
}	// extern "C"
#endif
// *****************************************************************************
//
// The vector table.
// This relies on the linker script to place at correct location in memory.
//
// *****************************************************************************
extern void(*const g_pfnVectors[]) (void);
__attribute__ ((section(".isr_vector")))
void(*const g_pfnVectors[]) (void) = {
	// Core Level - CM0plus
	&_vStackTop,// The initial stack pointer
	ResetISR,						// The reset handler
	NMI_Handler,					// The NMI handler
	HardFault_Handler,				// The hard fault handler
	0,								// Reserved
	0,								// Reserved
	0,								// Reserved
	0,								// Reserved
	0,								// Reserved
	0,								// Reserved
	0,								// Reserved
	SVC_Handler,					// SVCall handler
	0,								// Reserved
	0,								// Reserved
	PendSV_Handler,					// The PendSV handler
	SysTick_Handler,				// The SysTick handler

	// Chip Level - LPC8xx
	SPI0_IRQHandler,				// SPI0 controller
	SPI1_IRQHandler,				// SPI1 controller
	0,								// Reserved
	UART0_IRQHandler,				// UART0
	UART1_IRQHandler,				// UART1
	UART2_IRQHandler,				// UART2
	0,								// Reserved
	0,								// Reserved
	I2C_IRQHandler,					// I2C controller
	SCT_IRQHandler,					// Smart Counter Timer
	MRT_IRQHandler,					// Multi-Rate Timer
	CMP_IRQHandler,					// Comparator
	WDT_IRQHandler,					// Watchdog
	BOD_IRQHandler,					// Brown Out Detect
	FLASH_IRQHandler,				// FLASH
	WKT_IRQHandler,					// Wakeup timer
	0,								// Reserved
	0,								// Reserved
	0,								// Reserved
	0,								// Reserved
	0,								// Reserved
	0,								// Reserved
	0,								// Reserved
	0,								// Reserved
	PININT0_IRQHandler,				// PIO INT0
	PININT1_IRQHandler,				// PIO INT1
	PININT2_IRQHandler,				// PIO INT2
	PININT3_IRQHandler,				// PIO INT3
	PININT4_IRQHandler,				// PIO INT4
	PININT5_IRQHandler,				// PIO INT5
	PININT6_IRQHandler,				// PIO INT6
	PININT7_IRQHandler,				// PIO INT7
};	/* End of g_pfnVectors */

// *****************************************************************************
// Functions to carry out the initialization of RW and BSS data sections. These
// are written as separate functions rather than being inlined within the
// ResetISR() function in order to cope with MCUs with multiple banks of
// memory.
// *****************************************************************************
__attribute__ ((section(".after_vectors")))
void data_init(unsigned int romstart, unsigned int start, unsigned int len) {
	unsigned int *pulDest = (unsigned int *) start;
	unsigned int *pulSrc = (unsigned int *) romstart;
	unsigned int loop;
	for (loop = 0; loop < len; loop = loop + 4) {
		*pulDest++ = *pulSrc++;
	}
}

__attribute__ ((section(".after_vectors")))
void bss_init(unsigned int start, unsigned int len) {
	unsigned int *pulDest = (unsigned int *) start;
	unsigned int loop;
	for (loop = 0; loop < len; loop = loop + 4) {
		*pulDest++ = 0;
	}
}

// *****************************************************************************
// The following symbols are constructs generated by the linker, indicating
// the location of various points in the "Global Section Table". This table is
// created by the linker via the Code Red managed linker script mechanism. It
// contains the load address, execution address and length of each RW data
// section and the execution and length of each BSS (zero initialized) section.
// *****************************************************************************
extern unsigned int __data_section_table;
extern unsigned int __data_section_table_end;
extern unsigned int __bss_section_table;
extern unsigned int __bss_section_table_end;

// *****************************************************************************
// Reset entry point for your code.
// Sets up a simple runtime environment and initializes the C/C++
// library.
// *****************************************************************************
__attribute__ ((section(".after_vectors")))
void
ResetISR(void) {

	//
	// Copy the data sections from flash to SRAM.
	//
	unsigned int LoadAddr, ExeAddr, SectionLen;
	unsigned int *SectionTableAddr;

	// Load base address of Global Section Table
	SectionTableAddr = &__data_section_table;

	// Copy the data sections from flash to SRAM.
	while (SectionTableAddr < &__data_section_table_end) {
		LoadAddr = *SectionTableAddr++;
		ExeAddr = *SectionTableAddr++;
		SectionLen = *SectionTableAddr++;
		data_init(LoadAddr, ExeAddr, SectionLen);
	}
	// At this point, SectionTableAddr = &__bss_section_table;
	// Zero fill the bss segment
	while (SectionTableAddr < &__bss_section_table_end) {
		ExeAddr = *SectionTableAddr++;
		SectionLen = *SectionTableAddr++;
		bss_init(ExeAddr, SectionLen);
	}
	extern void SystemInit(void);

	SystemInit();

#if defined(__cplusplus)
	//
	// Call C++ library initialisation
	//
	__libc_init_array();
#endif

#if defined(__REDLIB__)
	// Call the Redlib library, which in turn calls main()
	__main();
#else
	main();
#endif

	//
	// main() shouldn't return, but if it does, we'll just enter an infinite loop
	//
	while (1) {}
}

// *****************************************************************************
// Default exception handlers. Override the ones here by defining your own
// handler routines in your application code.
// *****************************************************************************
__attribute__ ((section(".after_vectors")))
void NMI_Handler(void)
{
	while (1) {}
}

__attribute__ ((section(".after_vectors")))
void HardFault_Handler(void)
{
	while (1) {}
}

__attribute__ ((section(".after_vectors")))
void SVCall_Handler(void)
{
	while (1) {}
}

__attribute__ ((section(".after_vectors")))
void PendSV_Handler(void)
{
	while (1) {}
}

__attribute__ ((section(".after_vectors")))
void SysTick_Handler(void)
{
	while (1) {}
}

// *****************************************************************************
//
// Processor ends up here if an unexpected interrupt occurs or a specific
// handler is not present in the application code.
//
// *****************************************************************************
__attribute__ ((section(".after_vectors")))
void IntDefaultHandler(void)
{
	while (1) {}
}
