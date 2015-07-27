;/*****************************************************************************
; * @file:    startup_LPC11xx.s
; * @purpose: CMSIS Cortex-M0 Core Device Startup File 
; *           for the NXP LPC11xx Device Series 
; * @version: V1.00
; * @date:    19. October 2009
; *----------------------------------------------------------------------------
; *
; * Copyright (C) 2009 ARM Limited. All rights reserved.
; *
; * ARM Limited (ARM) is supplying this software for use with Cortex-Mx 
; * processor based microcontrollers.  This file can be freely distributed 
; * within development tools that are supporting such ARM based processors. 
; *
; * THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
; * OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
; * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
; * ARM SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR
; * CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
; *
; ******************************************************************************/


;
; The modules in this file are included in the libraries, and may be replaced
; by any user-defined modules that define the PUBLIC symbol _program_start or
; a user defined start symbol.
; To override the cstartup defined in the library, simply add your modified
; version to the workbench project.
;
; The vector table is normally located at address 0.
; When debugging in RAM, it can be located in RAM, aligned to at least 2^6.
; The name "__vector_table" has special meaning for C-SPY:
; it is where the SP start value is found, and the NVIC vector
; table register (VTOR) is initialized to this address if != 0.
;
; Cortex-M version
;

#include "sys_config.h"

        MODULE  ?cstartup

        ;; Forward declaration of sections.
        SECTION CSTACK:DATA:NOROOT(3)

        SECTION .intvec:CODE:NOROOT(2)
	
        EXTERN  __iar_program_start
        EXTERN  SystemInit
        EXTERN  xPortSysTickHandler
        EXTERN  xPortPendSVHandler
        EXTERN  vPortSVCHandler		
        PUBLIC  __vector_table
        PUBLIC  __vector_table_0x1c
        PUBLIC  __Vectors
        PUBLIC  __Vectors_End
        PUBLIC  __Vectors_Size

        DATA

__vector_table
        DCD     sfe(CSTACK)
        DCD     Reset_Handler

        DCD     NMI_Handler
        DCD     HardFault_Handler
        DCD     MemManage_Handler
        DCD     BusFault_Handler
        DCD     UsageFault_Handler
__vector_table_0x1c
        DCD     0
        DCD     0
        DCD     0
        DCD     0
        DCD     vPortSVCHandler           ; FreeRTOS SVCall Handler
        DCD     DebugMon_Handler
        DCD     0
        DCD     xPortPendSVHandler        ; FreeRTOS PendSV Handler
        DCD     xPortSysTickHandler       ; FreeRTOS SysTick Handler

#ifdef CHIP_LPC110X
        ; Wakeup sources for the I/O pins:
        ;   PIO0 (0:11)
        ;   PIO1 (0)
        DCD WAKEUP_IRQHandler ; PIO0_0  Wakeup
        DCD Reserved_IRQHandler
        DCD Reserved_IRQHandler
        DCD Reserved_IRQHandler
        DCD Reserved_IRQHandler
        DCD Reserved_IRQHandler
        DCD Reserved_IRQHandler
        DCD Reserved_IRQHandler
        DCD WAKEUP_IRQHandler ; PIO0_8  Wakeup
        DCD WAKEUP_IRQHandler ; PIO0_9  Wakeup
        DCD WAKEUP_IRQHandler ; PIO0_10 Wakeup
        DCD WAKEUP_IRQHandler ; PIO0_11 Wakeup
        DCD WAKEUP_IRQHandler ; PIO1_0  Wakeup
        DCD Reserved_IRQHandler
        DCD Reserved_IRQHandler
        DCD Reserved_IRQHandler
        DCD TIMER16_0_IRQHandler ; CT16B0 (16-bit Timer 0)
        DCD TIMER16_1_IRQHandler ; CT16B1 (16-bit Timer 1)
        DCD TIMER32_0_IRQHandler ; CT32B0 (32-bit Timer 0)
        DCD TIMER32_1_IRQHandler ; CT32B0 (32-bit Timer 1)
        DCD SSP0_IRQHandler ; SPI/SSP0 Interrupt
        DCD UART_IRQHandler ; UART0
        DCD Reserved_IRQHandler
        DCD Reserved_IRQHandler
        DCD ADC_IRQHandler ; ADC   (A/D Converter)
        DCD WDT_IRQHandler ; WDT   (Watchdog Timer)
        DCD BOD_IRQHandler ; BOD   (Brownout Detect)
        DCD Reserved_IRQHandler
        DCD Reserved_IRQHandler
        DCD Reserved_IRQHandler
        DCD PIOINT1_IRQHandler ; PIO INT1
        DCD PIOINT0_IRQHandler ; PIO INT0
#endif

#ifdef CHIP_LPC11XXLV
        ; Wakeup sources for the I/O pins:
        ;   PIO0 (0:11)
        ;   PIO1 (0)
        DCD WAKEUP_IRQHandler ; PIO0_0  Wakeup
        DCD WAKEUP_IRQHandler ; PIO0_1  Wakeup
        DCD WAKEUP_IRQHandler ; PIO0_2  Wakeup
        DCD WAKEUP_IRQHandler ; PIO0_3  Wakeup
        DCD WAKEUP_IRQHandler ; PIO0_4  Wakeup
        DCD WAKEUP_IRQHandler ; PIO0_5  Wakeup
        DCD WAKEUP_IRQHandler ; PIO0_6  Wakeup
        DCD WAKEUP_IRQHandler ; PIO0_7  Wakeup
        DCD WAKEUP_IRQHandler ; PIO0_8  Wakeup
        DCD WAKEUP_IRQHandler ; PIO0_9  Wakeup
        DCD WAKEUP_IRQHandler ; PIO0_10 Wakeup
        DCD WAKEUP_IRQHandler ; PIO0_11 Wakeup
        DCD WAKEUP_IRQHandler ; PIO1_0  Wakeup
        DCD Reserved_IRQHandler
        DCD Reserved_IRQHandler
        DCD I2C_IRQHandler ; I2C0
        DCD TIMER16_0_IRQHandler ; CT16B0 (16-bit Timer 0)
        DCD TIMER16_1_IRQHandler ; CT16B1 (16-bit Timer 1)
        DCD TIMER32_0_IRQHandler ; CT32B0 (32-bit Timer 0)
        DCD TIMER32_1_IRQHandler ; CT32B1 (32-bit Timer 1)
        DCD SSP0_IRQHandler ; SPI/SSP0 Interrupt
        DCD UART_IRQHandler ; UART0
        DCD Reserved_IRQHandler
        DCD Reserved_IRQHandler
        DCD ADC_IRQHandler ; ADC   (A/D Converter)
        DCD WDT_IRQHandler ; WDT   (Watchdog Timer)
        DCD BOD_IRQHandler ; BOD   (Brownout Detect)
        DCD FMC_IRQHandler ; 16+27: IP2111 Flash Memory Controller
        DCD PIOINT3_IRQHandler ; PIO INT3
        DCD PIOINT2_IRQHandler ; PIO INT2
        DCD PIOINT1_IRQHandler ; PIO INT1
        DCD PIOINT0_IRQHandler ; PIO INT0
#endif

#ifdef CHIP_LPC11AXX
        ; LPC11A specific handlers
        DCD PIN_INT0_IRQHandler ; 16+ 0: Pin interrupt
        DCD PIN_INT1_IRQHandler ; 16+ 1: Pin interrupt
        DCD PIN_INT2_IRQHandler ; 16+ 2: Pin interrupt
        DCD PIN_INT3_IRQHandler ; 16+ 3: Pin interrupt
        DCD PIN_INT4_IRQHandler ; 16+ 4: Pin interrupt
        DCD PIN_INT5_IRQHandler ; 16+ 5: Pin interrupt
        DCD PIN_INT6_IRQHandler ; 16+ 6: Pin interrupt
        DCD PIN_INT7_IRQHandler ; 16+ 7: Pin interrupt
        DCD GINT0_IRQHandler ; 16+ 8: Port interrupt
        DCD GINT1_IRQHandler ; 16+ 9: Port interrupt
        DCD ACMP_IRQHandler ; 16+10: Analog Comparator
        DCD DAC_IRQHandler ; 16+11: D/A Converter
        DCD Reserved_IRQHandler
        DCD Reserved_IRQHandler
        DCD SSP1_IRQHandler ; 16+14: SSP1
        DCD I2C_IRQHandler ; 16+15: I2C
        DCD TIMER16_0_IRQHandler ; 16+16: 16-bit Timer0
        DCD TIMER16_1_IRQHandler ; 16+17: 16-bit Timer1
        DCD TIMER32_0_IRQHandler ; 16+18: 32-bit Timer0
        DCD TIMER32_1_IRQHandler ; 16+19: 32-bit Timer1
        DCD SSP0_IRQHandler ; 16+20: SSP0
        DCD USART_IRQHandler ; 16+21: USART
        DCD Reserved_IRQHandler
        DCD Reserved_IRQHandler
        DCD ADC_IRQHandler ; 16+24: A/D Converter
        DCD WDT_IRQHandler ; 16+25: Watchdog Timer
        DCD BOD_IRQHandler ; 16+26: Brown Out Detect
        DCD FMC_IRQHandler ; 16+27: IP2111 Flash Memory Controller
        DCD Reserved_IRQHandler
        DCD Reserved_IRQHandler
        DCD Reserved_IRQHandler
        DCD Reserved_IRQHandler
#endif

#ifdef CHIP_LPC11CXX
        ; Wakeup sources for the I/O pins:
        ;    PIO0 (0:11)
        ;    PIO1 (0)
        DCD WAKEUP_IRQHandler ; PIO0_0  Wakeup
        DCD WAKEUP_IRQHandler ; PIO0_1  Wakeup
        DCD WAKEUP_IRQHandler ; PIO0_2  Wakeup
        DCD WAKEUP_IRQHandler ; PIO0_3  Wakeup
        DCD WAKEUP_IRQHandler ; PIO0_4  Wakeup
        DCD WAKEUP_IRQHandler ; PIO0_5  Wakeup
        DCD WAKEUP_IRQHandler ; PIO0_6  Wakeup
        DCD WAKEUP_IRQHandler ; PIO0_7  Wakeup
        DCD WAKEUP_IRQHandler ; PIO0_8  Wakeup
        DCD WAKEUP_IRQHandler ; PIO0_9  Wakeup
        DCD WAKEUP_IRQHandler ; PIO0_10 Wakeup
        DCD WAKEUP_IRQHandler ; PIO0_11 Wakeup
        DCD WAKEUP_IRQHandler ; PIO1_0  Wakeup
        DCD CAN_IRQHandler ; C_CAN Interrupt
        DCD SSP1_IRQHandler ; SPI/SSP1 Interrupt
        DCD I2C_IRQHandler ; I2C0
        DCD TIMER16_0_IRQHandler ; CT16B0 (16-bit Timer 0)
        DCD TIMER16_1_IRQHandler ; CT16B1 (16-bit Timer 1)
        DCD TIMER32_0_IRQHandler ; CT32B0 (32-bit Timer 0)
        DCD TIMER32_1_IRQHandler ; CT32B1 (32-bit Timer 1)
        DCD SSP0_IRQHandler ; SPI/SSP0 Interrupt
        DCD UART_IRQHandler ; UART0
        DCD Reserved_IRQHandler
        DCD Reserved_IRQHandler
        DCD ADC_IRQHandler ; ADC   (A/D Converter)
        DCD WDT_IRQHandler ; WDT   (Watchdog Timer)
        DCD BOD_IRQHandler ; BOD   (Brownout Detect)
        DCD Reserved_IRQHandler
        DCD PIOINT3_IRQHandler ; PIO INT3
        DCD PIOINT2_IRQHandler ; PIO INT2
        DCD PIOINT1_IRQHandler ; PIO INT1
        DCD PIOINT0_IRQHandler ; PIO INT0
#endif

#ifdef CHIP_LPC11EXX
        ; LPC11E specific handlers
        DCD FLEX_INT0_IRQHandler ; 0 - GPIO pin interrupt 0
        DCD FLEX_INT1_IRQHandler ; 1 - GPIO pin interrupt 1
        DCD FLEX_INT2_IRQHandler ; 2 - GPIO pin interrupt 2
        DCD FLEX_INT3_IRQHandler ; 3 - GPIO pin interrupt 3
        DCD FLEX_INT4_IRQHandler ; 4 - GPIO pin interrupt 4
        DCD FLEX_INT5_IRQHandler ; 5 - GPIO pin interrupt 5
        DCD FLEX_INT6_IRQHandler ; 6 - GPIO pin interrupt 6
        DCD FLEX_INT7_IRQHandler ; 7 - GPIO pin interrupt 7
        DCD GINT0_IRQHandler ; 8 - GPIO GROUP0 interrupt
        DCD GINT1_IRQHandler ; 9 - GPIO GROUP1 interrupt
        DCD Reserved_IRQHandler
        DCD Reserved_IRQHandler
        DCD Reserved_IRQHandler
        DCD Reserved_IRQHandler
        DCD SSP1_IRQHandler ; 14 - SPI/SSP1 Interrupt
        DCD I2C_IRQHandler ; 15 - I2C0
        DCD TIMER16_0_IRQHandler ; 16 - CT16B0 (16-bit Timer 0)
        DCD TIMER16_1_IRQHandler ; 17 - CT16B1 (16-bit Timer 1)
        DCD TIMER32_0_IRQHandler ; 18 - CT32B0 (32-bit Timer 0)
        DCD TIMER32_1_IRQHandler ; 19 - CT32B1 (32-bit Timer 1)
        DCD SSP0_IRQHandler ; 20 - SPI/SSP0 Interrupt
        DCD UART_IRQHandler ; 21 - UART0
        DCD Reserved_IRQHandler
        DCD Reserved_IRQHandler
        DCD ADC_IRQHandler ; 24 - ADC (A/D Converter)
        DCD WDT_IRQHandler ; 25 - WDT (Watchdog Timer)
        DCD BOD_IRQHandler ; 26 - BOD (Brownout Detect)
        DCD FMC_IRQHandler ; 27 - IP2111 Flash Memory Controller
        DCD Reserved_IRQHandler
        DCD Reserved_IRQHandler
        DCD Reserved_IRQHandler
        DCD Reserved_IRQHandler
#endif

#ifdef CHIP_LPC11UXX
        ;  LPC11U specific handlers
        DCD FLEX_INT0_IRQHandler ; 0 - GPIO pin interrupt 0
        DCD FLEX_INT1_IRQHandler ; 1 - GPIO pin interrupt 1
        DCD FLEX_INT2_IRQHandler ; 2 - GPIO pin interrupt 2
        DCD FLEX_INT3_IRQHandler ; 3 - GPIO pin interrupt 3
        DCD FLEX_INT4_IRQHandler ; 4 - GPIO pin interrupt 4
        DCD FLEX_INT5_IRQHandler ; 5 - GPIO pin interrupt 5
        DCD FLEX_INT6_IRQHandler ; 6 - GPIO pin interrupt 6
        DCD FLEX_INT7_IRQHandler ; 7 - GPIO pin interrupt 7
        DCD GINT0_IRQHandler ; 8 - GPIO GROUP0 interrupt
        DCD GINT1_IRQHandler ; 9 - GPIO GROUP1 interrupt
        DCD Reserved_IRQHandler
        DCD Reserved_IRQHandler
        DCD Reserved_IRQHandler
        DCD Reserved_IRQHandler
        DCD SSP1_IRQHandler ; 14 - SPI/SSP1 Interrupt
        DCD I2C_IRQHandler ; 15 - I2C0
        DCD TIMER16_0_IRQHandler ; 16 - CT16B0 (16-bit Timer 0)
        DCD TIMER16_1_IRQHandler ; 17 - CT16B1 (16-bit Timer 1)
        DCD TIMER32_0_IRQHandler ; 18 - CT32B0 (32-bit Timer 0)
        DCD TIMER32_1_IRQHandler ; 19 - CT32B1 (32-bit Timer 1)
        DCD SSP0_IRQHandler ; 20 - SPI/SSP0 Interrupt
        DCD UART_IRQHandler ; 21 - UART0
        DCD USB_IRQHandler ; 22 - USB IRQ
        DCD USB_FIQHandler ; 23 - USB FIQ
        DCD ADC_IRQHandler ; 24 - ADC (A/D Converter)
        DCD WDT_IRQHandler ; 25 - WDT (Watchdog Timer)
        DCD BOD_IRQHandler ; 26 - BOD (Brownout Detect)
        DCD FMC_IRQHandler ; 27 - IP2111 Flash Memory Controller
        DCD Reserved_IRQHandler
        DCD Reserved_IRQHandler
        DCD USBWakeup_IRQHandler ; 30 - USB wake-up interrupt
        DCD Reserved_IRQHandler
#endif

__Vectors_End

__Vectors       EQU   __vector_table
__Vectors_Size 	EQU 	__Vectors_End - __Vectors


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;
;; Default interrupt handlers.
;;
        THUMB

        PUBWEAK Reset_Handler
        SECTION .text:CODE:REORDER(2)
Reset_Handler
        LDR     R0, =SystemInit
        BLX     R0
        LDR     R0, =__iar_program_start
        BX      R0

        PUBWEAK NMI_Handler
        SECTION .text:CODE:REORDER(1)
NMI_Handler
        B NMI_Handler

        PUBWEAK HardFault_Handler
        SECTION .text:CODE:REORDER(1)
HardFault_Handler
        B HardFault_Handler

        PUBWEAK MemManage_Handler
        SECTION .text:CODE:REORDER(1)
MemManage_Handler
        B MemManage_Handler

        PUBWEAK BusFault_Handler
        SECTION .text:CODE:REORDER(1)
BusFault_Handler
        B BusFault_Handler

        PUBWEAK UsageFault_Handler
        SECTION .text:CODE:REORDER(1)
UsageFault_Handler
        B UsageFault_Handler

        PUBWEAK SVC_Handler
        SECTION .text:CODE:REORDER(1)
SVC_Handler
        B SVC_Handler

        PUBWEAK DebugMon_Handler
        SECTION .text:CODE:REORDER(1)
DebugMon_Handler
        B DebugMon_Handler

        PUBWEAK PendSV_Handler
        SECTION .text:CODE:REORDER(1)
PendSV_Handler
        B PendSV_Handler

        PUBWEAK SysTick_Handler
        SECTION .text:CODE:REORDER(1)
SysTick_Handler
        B SysTick_Handler

	PUBWEAK Reserved_IRQHandler
        SECTION .text:CODE:REORDER(1)
Reserved_IRQHandler
        B Reserved_IRQHandler

Default_Handler:
        B Default_Handler

#ifdef CHIP_LPC110X
        PUBWEAK  WAKEUP_IRQHandler
        PUBWEAK	TIMER16_0_IRQHandler
        PUBWEAK	TIMER16_1_IRQHandler
        PUBWEAK	TIMER32_0_IRQHandler
        PUBWEAK	TIMER32_1_IRQHandler
        PUBWEAK	SSP0_IRQHandler
        PUBWEAK	UART_IRQHandler
        PUBWEAK	ADC_IRQHandler
        PUBWEAK	WDT_IRQHandler
        PUBWEAK	BOD_IRQHandler
        PUBWEAK	PIOINT1_IRQHandler
        PUBWEAK	PIOINT0_IRQHandler

WAKEUP_IRQHandler
TIMER16_0_IRQHandler
TIMER16_1_IRQHandler
TIMER32_0_IRQHandler
TIMER32_1_IRQHandler
SSP0_IRQHandler
UART_IRQHandler
ADC_IRQHandler
WDT_IRQHandler
BOD_IRQHandler
PIOINT1_IRQHandler
PIOINT0_IRQHandler
#endif

#ifdef CHIP_LPC11XXLV
        PUBWEAK  WAKEUP_IRQHandler         
        PUBWEAK  I2C_IRQHandler            
        PUBWEAK	TIMER16_0_IRQHandler      
        PUBWEAK	TIMER16_1_IRQHandler      
        PUBWEAK	TIMER32_0_IRQHandler      
        PUBWEAK	TIMER32_1_IRQHandler      
        PUBWEAK	SSP0_IRQHandler           
        PUBWEAK	UART_IRQHandler           
        PUBWEAK	ADC_IRQHandler            
        PUBWEAK	WDT_IRQHandler            
        PUBWEAK	BOD_IRQHandler            
        PUBWEAK	FMC_IRQHandler            
        PUBWEAK	PIOINT3_IRQHandler        
        PUBWEAK	PIOINT2_IRQHandler        
        PUBWEAK	PIOINT1_IRQHandler        
        PUBWEAK	PIOINT0_IRQHandler        


WAKEUP_IRQHandler
I2C_IRQHandler
TIMER16_0_IRQHandler
TIMER16_1_IRQHandler
TIMER32_0_IRQHandler
TIMER32_1_IRQHandler
SSP0_IRQHandler
UART_IRQHandler
ADC_IRQHandler
WDT_IRQHandler
BOD_IRQHandler
FMC_IRQHandler
PIOINT3_IRQHandler
PIOINT2_IRQHandler
PIOINT1_IRQHandler
PIOINT0_IRQHandler
#endif

#ifdef CHIP_LPC11AXX
        PUBWEAK  PIN_INT0_IRQHandler       
        PUBWEAK  PIN_INT1_IRQHandler       
        PUBWEAK  PIN_INT2_IRQHandler       
        PUBWEAK  PIN_INT3_IRQHandler       
        PUBWEAK  PIN_INT4_IRQHandler       
        PUBWEAK  PIN_INT5_IRQHandler       
        PUBWEAK  PIN_INT6_IRQHandler       
        PUBWEAK  PIN_INT7_IRQHandler       
        PUBWEAK  GINT0_IRQHandler          
        PUBWEAK  GINT1_IRQHandler          
        PUBWEAK  ACMP_IRQHandler           
        PUBWEAK  DAC_IRQHandler            
        PUBWEAK  SSP1_IRQHandler           
        PUBWEAK  I2C_IRQHandler            
        PUBWEAK	TIMER16_0_IRQHandler      
        PUBWEAK	TIMER16_1_IRQHandler      
        PUBWEAK	TIMER32_0_IRQHandler      
        PUBWEAK	TIMER32_1_IRQHandler      
        PUBWEAK	SSP0_IRQHandler           
        PUBWEAK	UART_IRQHandler           
        PUBWEAK	ADC_IRQHandler            
        PUBWEAK	WDT_IRQHandler            
        PUBWEAK	BOD_IRQHandler            
        PUBWEAK	FMC_IRQHandler            

PIN_INT0_IRQHandler
PIN_INT1_IRQHandler
PIN_INT2_IRQHandler
PIN_INT3_IRQHandler
PIN_INT4_IRQHandler
PIN_INT5_IRQHandler
PIN_INT6_IRQHandler
PIN_INT7_IRQHandler
GINT0_IRQHandler
GINT1_IRQHandler
ACMP_IRQHandler
DAC_IRQHandler
SSP1_IRQHandler
I2C_IRQHandler
TIMER16_0_IRQHandler
TIMER16_1_IRQHandler
TIMER32_0_IRQHandler
TIMER32_1_IRQHandler
SSP0_IRQHandler
UART_IRQHandler
ADC_IRQHandler
WDT_IRQHandler
BOD_IRQHandler
FMC_IRQHandler
#endif

#ifdef CHIP_LPC11CXX
        PUBWEAK  WAKEUP_IRQHandler         
        PUBWEAK  CAN_IRQHandler            
        PUBWEAK  SSP1_IRQHandler           
        PUBWEAK  I2C_IRQHandler            
        PUBWEAK	TIMER16_0_IRQHandler      
        PUBWEAK	TIMER16_1_IRQHandler      
        PUBWEAK	TIMER32_0_IRQHandler      
        PUBWEAK	TIMER32_1_IRQHandler      
        PUBWEAK	SSP0_IRQHandler           
        PUBWEAK	UART_IRQHandler           
        PUBWEAK	ADC_IRQHandler            
        PUBWEAK	WDT_IRQHandler            
        PUBWEAK	BOD_IRQHandler            
        PUBWEAK	PIOINT3_IRQHandler        
        PUBWEAK	PIOINT2_IRQHandler        
        PUBWEAK	PIOINT1_IRQHandler        
        PUBWEAK	PIOINT0_IRQHandler        

WAKEUP_IRQHandler
CAN_IRQHandler
SSP1_IRQHandler
I2C_IRQHandler
TIMER16_0_IRQHandler
TIMER16_1_IRQHandler
TIMER32_0_IRQHandler
TIMER32_1_IRQHandler
SSP0_IRQHandler
UART_IRQHandler
ADC_IRQHandler
WDT_IRQHandler
BOD_IRQHandler
PIOINT3_IRQHandler
PIOINT2_IRQHandler
PIOINT1_IRQHandler
PIOINT0_IRQHandler
#endif

#ifdef CHIP_LPC11EXX
        PUBWEAK  FLEX_INT0_IRQHandler      
        PUBWEAK  FLEX_INT1_IRQHandler      
        PUBWEAK  FLEX_INT2_IRQHandler      
        PUBWEAK  FLEX_INT3_IRQHandler      
        PUBWEAK  FLEX_INT4_IRQHandler      
        PUBWEAK  FLEX_INT5_IRQHandler      
        PUBWEAK  FLEX_INT6_IRQHandler      
        PUBWEAK  FLEX_INT7_IRQHandler      
        PUBWEAK  GINT0_IRQHandler          
        PUBWEAK  GINT1_IRQHandler          
        PUBWEAK  SSP1_IRQHandler           
        PUBWEAK  I2C_IRQHandler            
        PUBWEAK	TIMER16_0_IRQHandler      
        PUBWEAK	TIMER16_1_IRQHandler      
        PUBWEAK	TIMER32_0_IRQHandler      
        PUBWEAK	TIMER32_1_IRQHandler      
        PUBWEAK	SSP0_IRQHandler           
        PUBWEAK	UART_IRQHandler           
        PUBWEAK	ADC_IRQHandler            
        PUBWEAK	WDT_IRQHandler            
        PUBWEAK	BOD_IRQHandler            
        PUBWEAK	FMC_IRQHandler            

FLEX_INT0_IRQHandler
FLEX_INT1_IRQHandler
FLEX_INT2_IRQHandler
FLEX_INT3_IRQHandler
FLEX_INT4_IRQHandler
FLEX_INT5_IRQHandler
FLEX_INT6_IRQHandler
FLEX_INT7_IRQHandler
GINT0_IRQHandler
GINT1_IRQHandler
SSP1_IRQHandler
I2C_IRQHandler
TIMER16_0_IRQHandler
TIMER16_1_IRQHandler
TIMER32_0_IRQHandler
TIMER32_1_IRQHandler
SSP0_IRQHandler
UART_IRQHandler
ADC_IRQHandler
WDT_IRQHandler
BOD_IRQHandler
FMC_IRQHandler
#endif

#ifdef CHIP_LPC11UXX
        PUBWEAK  FLEX_INT0_IRQHandler      
        PUBWEAK  FLEX_INT1_IRQHandler      
        PUBWEAK  FLEX_INT2_IRQHandler      
        PUBWEAK  FLEX_INT3_IRQHandler      
        PUBWEAK  FLEX_INT4_IRQHandler      
        PUBWEAK  FLEX_INT5_IRQHandler      
        PUBWEAK  FLEX_INT6_IRQHandler      
        PUBWEAK  FLEX_INT7_IRQHandler      
        PUBWEAK  GINT0_IRQHandler          
        PUBWEAK  GINT1_IRQHandler          
        PUBWEAK  SSP1_IRQHandler           
        PUBWEAK  I2C_IRQHandler            
        PUBWEAK	TIMER16_0_IRQHandler      
        PUBWEAK	TIMER16_1_IRQHandler      
        PUBWEAK	TIMER32_0_IRQHandler      
        PUBWEAK	TIMER32_1_IRQHandler      
        PUBWEAK	SSP0_IRQHandler           
        PUBWEAK	UART_IRQHandler           
        PUBWEAK	USB_IRQHandler            
        PUBWEAK	USB_FIQHandler            
        PUBWEAK	ADC_IRQHandler            
        PUBWEAK	WDT_IRQHandler            
        PUBWEAK	BOD_IRQHandler            
        PUBWEAK	FMC_IRQHandler            
        PUBWEAK	USBWakeup_IRQHandler      

FLEX_INT0_IRQHandler
FLEX_INT1_IRQHandler
FLEX_INT2_IRQHandler
FLEX_INT3_IRQHandler
FLEX_INT4_IRQHandler
FLEX_INT5_IRQHandler
FLEX_INT6_IRQHandler
FLEX_INT7_IRQHandler
GINT0_IRQHandler
GINT1_IRQHandler
SSP1_IRQHandler
I2C_IRQHandler
TIMER16_0_IRQHandler
TIMER16_1_IRQHandler
TIMER32_0_IRQHandler
TIMER32_1_IRQHandler
SSP0_IRQHandler
UART_IRQHandler
USB_IRQHandler
USB_FIQHandler
ADC_IRQHandler
WDT_IRQHandler
BOD_IRQHandler
FMC_IRQHandler
USBWakeup_IRQHandler
#endif

        END
