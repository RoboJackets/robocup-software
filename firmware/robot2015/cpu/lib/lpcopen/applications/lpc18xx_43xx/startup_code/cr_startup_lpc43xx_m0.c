// *****************************************************************************
//   +--+
//   | ++----+
//   +-++    |
//     |     |
//   +-+--+  |
//   | +--+--+
//   +----+    Copyright (c) 2011-12 Code Red Technologies Ltd.
//
// LPC43xx Microcontroller Startup code for use with Red Suite
//
// Version : 120430
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

// Code Red - if CMSIS is being used, then SystemInit() routine
// will be called by startup code rather than in application's main()
extern void SystemInit(void);

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
WEAK void MemManage_Handler(void);
WEAK void BusFault_Handler(void);
WEAK void UsageFault_Handler(void);
WEAK void SVC_Handler(void);
WEAK void DebugMon_Handler(void);
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
void RTC_IRQHandler(void) ALIAS(IntDefaultHandler);
void MX_CORE_IRQHandler(void) ALIAS(IntDefaultHandler);
void DMA_IRQHandler(void) ALIAS(IntDefaultHandler);
void FLASHEEPROM_IRQHandler(void) ALIAS(IntDefaultHandler);
void ETH_IRQHandler(void) ALIAS(IntDefaultHandler);
void SDIO_IRQHandler(void) ALIAS(IntDefaultHandler);
void LCD_IRQHandler(void) ALIAS(IntDefaultHandler);
void USB0_IRQHandler(void) ALIAS(IntDefaultHandler);
void USB1_IRQHandler(void) ALIAS(IntDefaultHandler);
void SCT_IRQHandler(void) ALIAS(IntDefaultHandler);
void RIT_IRQHandler(void) ALIAS(IntDefaultHandler);
void TIMER0_IRQHandler(void) ALIAS(IntDefaultHandler);
void GINT1_IRQHandler(void) ALIAS(IntDefaultHandler);
void GPIO4_IRQHandler(void) ALIAS(IntDefaultHandler);
void TIMER3_IRQHandler(void) ALIAS(IntDefaultHandler);
void MCPWM_IRQHandler(void) ALIAS(IntDefaultHandler);
void ADC0_IRQHandler(void) ALIAS(IntDefaultHandler);
void I2C0_IRQHandler(void) ALIAS(IntDefaultHandler);
void SGPIO_IRQHandler(void) ALIAS(IntDefaultHandler);
void SPI_IRQHandler (void) ALIAS(IntDefaultHandler);
void ADC1_IRQHandler(void) ALIAS(IntDefaultHandler);
void SSP0_IRQHandler(void) ALIAS(IntDefaultHandler);
void EVRT_IRQHandler(void) ALIAS(IntDefaultHandler);
void UART0_IRQHandler(void) ALIAS(IntDefaultHandler);
void UART1_IRQHandler(void) ALIAS(IntDefaultHandler);
void UART2_IRQHandler(void) ALIAS(IntDefaultHandler);
void UART3_IRQHandler(void) ALIAS(IntDefaultHandler);
void I2S0_IRQHandler(void) ALIAS(IntDefaultHandler);
void CAN0_IRQHandler(void) ALIAS(IntDefaultHandler);

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
	// Core Level - CM4/CM3
	&_vStackTop,	                // The initial stack pointer
	ResetISR,						// The reset handler
	NMI_Handler,					// The NMI handler
	HardFault_Handler,				// The hard fault handler
	MemManage_Handler,				// The MPU fault handler
	BusFault_Handler,				// The bus fault handler
	UsageFault_Handler,				// The usage fault handler
	0,								// Reserved
	0,								// Reserved
	0,								// Reserved
	0,								// Reserved
	SVC_Handler,					// SVCall handler
	DebugMon_Handler,				// Debug monitor handler
	0,								// Reserved
	PendSV_Handler,					// The PendSV handler
	SysTick_Handler,				// The SysTick handler

	// Chip Level - 43xx M0 core
	RTC_IRQHandler,					// 16 RTC
	MX_CORE_IRQHandler,				// 17 CortexM4/M0 (LPC43XX ONLY)
	DMA_IRQHandler,					// 18 General Purpose DMA
	0,								// 19 Reserved
	FLASHEEPROM_IRQHandler,			// 20 ORed flash Bank A, flash Bank B, EEPROM interrupts
	ETH_IRQHandler,					// 21 Ethernet
	SDIO_IRQHandler,				// 22 SD/MMC
	LCD_IRQHandler,					// 23 LCD
	USB0_IRQHandler,				// 24 USB0
	USB1_IRQHandler,				// 25 USB1
	SCT_IRQHandler,					// 26 State Configurable Timer
	RIT_IRQHandler,					// 27 ORed Repetitive Interrupt Timer, WWDT
	TIMER0_IRQHandler,				// 28 Timer0
	GINT1_IRQHandler,				// 29 GINT1
	GPIO4_IRQHandler,				// 30 GPIO4
	TIMER3_IRQHandler,				// 31 Timer 3
	MCPWM_IRQHandler,				// 32 Motor Control PWM
	ADC0_IRQHandler,				// 33 A/D Converter 0
	I2C0_IRQHandler,				// 34 ORed I2C0, I2C1
	SGPIO_IRQHandler,				// 35 SGPIO (LPC43XX ONLY)
	SPI_IRQHandler,					// 36 ORed SPI, DAC (LPC43XX ONLY)
	ADC1_IRQHandler,				// 37 A/D Converter 1
	SSP0_IRQHandler,				// 38 ORed SSP0, SSP1 
	EVRT_IRQHandler,				// 39 Event Router
	UART0_IRQHandler,				// 40 UART0
	UART1_IRQHandler,				// 41 UART1
	UART2_IRQHandler,				// 42 UART2
	UART3_IRQHandler,				// 43 USRT3
	I2S0_IRQHandler,				// 44 ORed I2S0, I2S1
	CAN0_IRQHandler,				// 45 C_CAN0
	0,								// 46 Reserved
	0,				                // 47 Reserved 
};

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
	for (loop = 0; loop < len; loop = loop + 4)
		*pulDest++ = *pulSrc++;
}

__attribute__ ((section(".after_vectors")))
void bss_init(unsigned int start, unsigned int len) {
	unsigned int *pulDest = (unsigned int *) start;
	unsigned int loop;
	for (loop = 0; loop < len; loop = loop + 4)
		*pulDest++ = 0;
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
//
// *****************************************************************************
void
ResetISR(void) {

	//
	// Copy the data sections from flash to SRAM.
	//
	unsigned int LoadAddr, ExeAddr, SectionLen;
	unsigned int *SectionTableAddr;

	/* Call SystemInit() for clocking/memory setup prior to scatter load */
	SystemInit();

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
void MemManage_Handler(void)
{
	while (1) {}
}

__attribute__ ((section(".after_vectors")))
void BusFault_Handler(void)
{
	while (1) {}
}

__attribute__ ((section(".after_vectors")))
void UsageFault_Handler(void)
{
	while (1) {}
}

__attribute__ ((section(".after_vectors")))
void SVC_Handler(void)
{
	while (1) {}
}

__attribute__ ((section(".after_vectors")))
void DebugMon_Handler(void)
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



