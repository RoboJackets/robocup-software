/**************************************************
 *
 * Part one of the system initialization code, contains low-level
 * initialization, plain thumb variant.
 *
 * Copyright 2011 IAR Systems. All rights reserved.
 *
 * $Revision: 47876 $
 *
 **************************************************/

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


        MODULE  ?cstartup

        ;; Forward declaration of sections.
        SECTION CSTACK:DATA:NOROOT(3)

        SECTION .intvec:CODE:NOROOT(2)

        EXTERN  __iar_program_start 
        EXTERN  SystemInit
		EXTERN  __M0Signature        
        PUBLIC  __vector_table
        PUBLIC  __vector_table_0x1c
        PUBLIC  __Vectors
        PUBLIC  __Vectors_End
        PUBLIC  __Vectors_Size

		;; Magic for M0 Image
M0_Image_Magic  EQU     0xAA55DEAD
				
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
        DCD     M0_Image_Magic
        DCD     __M0Signature
        DCD     0
        DCD     SVC_Handler
        DCD     DebugMon_Handler
        DCD     0
        DCD     PendSV_Handler
        DCD     SysTick_Handler

        ; External Interrupts
        DCD 	RTC_IRQHandler              ; 16 RTC
        DCD		MX_CORE_IRQHandler          ; 17 CortexM4 (LPC43XX ONLY) 
        DCD		DMA_IRQHandler				; 18 General Purpose DMA
        DCD		0				            ; 19 Reserved
        DCD		FLASHEEPROM_IRQHandler		; 20 ORed flash bank A, flash bank B, EEPROM interrupts
        DCD		ETH_IRQHandler				; 21 Ethernet
        DCD		SDIO_IRQHandler				; 22 SD/MMC
        DCD		LCD_IRQHandler				; 23 LCD
        DCD		USB0_IRQHandler				; 24 USB0
        DCD		USB1_IRQHandler				; 25 USB1
        DCD		SCT_IRQHandler				; 26 State Configurable Timer
        DCD		RIT_IRQHandler				; 27 Ored Repetitive Interrupt Timer, WWDT 
        DCD		TIMER0_IRQHandler			; 28 Timer0
        DCD		GINT1_IRQHandler			; 29 GINT1
        DCD		GPIO4_IRQHandler			; 30 GPIO4
        DCD		TIMER3_IRQHandler			; 31 Timer3
        DCD		MCPWM_IRQHandler			; 32 Motor Control PWM
        DCD		ADC0_IRQHandler				; 33 A/D Converter 0
        DCD		I2C0_IRQHandler				; 34 ORed I2C0, I2C1
        DCD		SGPIO_IRQHandler			; 35 SGPIO (LPC43XX ONLY)
        DCD		SPI_IRQHandler				; 36 ORed SPI, DAC (LPC43XX ONLY)
        DCD		ADC1_IRQHandler				; 37 A/D Converter 1
        DCD		SSP0_IRQHandler				; 38 ORed SSP0, SSP1
        DCD		EVRT_IRQHandler             ; 39 Event Router
        DCD		UART0_IRQHandler			; 40 UART0
        DCD		UART1_IRQHandler			; 41 UART1
        DCD		UART2_IRQHandler			; 42 ORed UART2, C_CAN1
        DCD		UART3_IRQHandler			; 43 UART3
        DCD		I2S0_IRQHandler				; 44 ORed I2S0, I2S1, QEI
        DCD 	CAN0_IRQHandler             ; 45 C_CAN0
        DCD 	0                           ; 46
        DCD 	0                           ; 47
__Vectors_End

__Vectors       EQU   __vector_table
__Vectors_Size 	EQU   __Vectors_End - __Vectors

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;
;; Default interrupt handlers.
;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

        THUMB

        PUBWEAK Reset_Handler
        SECTION .text:CODE:REORDER(2)
Reset_Handler
        LDR     R0, =SystemInit
        BLX     R0
        LDR     R0, =__iar_program_start
        BX      R0
  
        PUBWEAK NMI_Handler
        PUBWEAK HardFault_Handler
        PUBWEAK MemManage_Handler
        PUBWEAK BusFault_Handler
        PUBWEAK UsageFault_Handler
        PUBWEAK SVC_Handler
        PUBWEAK DebugMon_Handler
        PUBWEAK PendSV_Handler
        PUBWEAK SysTick_Handler
        PUBWEAK RTC_IRQHandler
        PUBWEAK MX_CORE_IRQHandler
        PUBWEAK DMA_IRQHandler
        PUBWEAK FLASHEEPROM_IRQHandler
        PUBWEAK ETH_IRQHandler
        PUBWEAK SDIO_IRQHandler
        PUBWEAK LCD_IRQHandler
        PUBWEAK USB0_IRQHandler
        PUBWEAK USB1_IRQHandler
        PUBWEAK SCT_IRQHandler
        PUBWEAK RIT_IRQHandler
        PUBWEAK TIMER0_IRQHandler
        PUBWEAK GINT1_IRQHandler
        PUBWEAK GPIO4_IRQHandler
        PUBWEAK TIMER3_IRQHandler
        PUBWEAK MCPWM_IRQHandler
        PUBWEAK ADC0_IRQHandler
        PUBWEAK I2C0_IRQHandler
        PUBWEAK SGPIO_IRQHandler
        PUBWEAK SPI_IRQHandler
        PUBWEAK ADC1_IRQHandler
        PUBWEAK SSP0_IRQHandler
        PUBWEAK EVRT_IRQHandler
        PUBWEAK UART0_IRQHandler
        PUBWEAK UART1_IRQHandler
        PUBWEAK UART2_IRQHandler
        PUBWEAK UART3_IRQHandler
        PUBWEAK I2S0_IRQHandler
        PUBWEAK CAN0_IRQHandler
        SECTION .text:CODE:REORDER(1)
NMI_Handler
        B .
SVC_Handler
        B .
DebugMon_Handler
        B .
PendSV_Handler
        B .
SysTick_Handler
        B .
HardFault_Handler
        B .
MemManage_Handler
        B .
BusFault_Handler
        B .
UsageFault_Handler
RTC_IRQHandler
MX_CORE_IRQHandler
DMA_IRQHandler 
FLASHEEPROM_IRQHandler
ETH_IRQHandler
SDIO_IRQHandler
LCD_IRQHandler
USB0_IRQHandler
USB1_IRQHandler
SCT_IRQHandler
RIT_IRQHandler
TIMER0_IRQHandler
GINT1_IRQHandler
GPIO4_IRQHandler
TIMER3_IRQHandler
MCPWM_IRQHandler
ADC0_IRQHandler
I2C0_IRQHandler
SGPIO_IRQHandler
SPI_IRQHandler
ADC1_IRQHandler
SSP0_IRQHandler
EVRT_IRQHandler
UART0_IRQHandler
UART1_IRQHandler
UART2_IRQHandler
UART3_IRQHandler
I2S0_IRQHandler
CAN0_IRQHandler
Default_IRQHandler
        B .

/* CRP Section - not needed for flashless devices */

;;;        SECTION .crp:CODE:ROOT(2)
;;;        DATA
/* Code Read Protection
NO_ISP  0x4E697370 -  Prevents sampling of pin PIO0_1 for entering ISP mode
CRP1    0x12345678 - Write to RAM command cannot access RAM below 0x10000300.
                   - Copy RAM to flash command can not write to Sector 0.
                   - Erase command can erase Sector 0 only when all sectors
                     are selected for erase.
                   - Compare command is disabled.
                   - Read Memory command is disabled.
CRP2    0x87654321 - Read Memory is disabled.
                   - Write to RAM is disabled.
                   - "Go" command is disabled.
                   - Copy RAM to flash is disabled.
                   - Compare is disabled.
CRP3    0x43218765 - Access to chip via the SWD pins is disabled. ISP entry
                     by pulling PIO0_1 LOW is disabled if a valid user code is
                     present in flash sector 0.
Caution: If CRP3 is selected, no future factory testing can be
performed on the device.
*/
;;;	    DCD	0xFFFFFFFF
;;;

        END
