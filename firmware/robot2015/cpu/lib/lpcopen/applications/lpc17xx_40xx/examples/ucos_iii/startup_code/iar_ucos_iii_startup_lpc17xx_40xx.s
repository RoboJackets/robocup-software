;/***********************************************************************
; * Project: LPC17xx/40xx startup code
; *
; * Description: LPC17xx/40xx startup code
; *
; * Copyright(C) 2011, NXP Semiconductor
; * All rights reserved.
; *
; ***********************************************************************
; * Software that is described herein is for illustrative purposes only
; * which provides customers with programming information regarding the
; * products. This software is supplied "AS IS" without any warranties.
; * NXP Semiconductors assumes no responsibility or liability for the
; * use of the software, conveys no license or title under any patent,
; * copyright, or mask work right to the product. NXP Semiconductors
; * reserves the right to make changes in the software without
; * notification. NXP Semiconductors also make no representation or
; * warranty that such application will be suitable for the specified
; * use without further testing or modification.
; **********************************************************************/
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
		EXTERN	OS_CPU_SysTickHandler
		EXTERN	OS_CPU_PendSVHandler        
        PUBLIC  __vector_table
        PUBLIC  __vector_table_0x1c
        PUBLIC  __Vectors
        PUBLIC  __Vectors_End
        PUBLIC  __Vectors_Size

        DATA
Sign_Value      EQU	   0x5A5A5A5A
__vector_table
        DCD     sfe(CSTACK)              	; 0 Top of Stack
        DCD     Reset_Handler             	; 1 Reset Handler

        DCD     NMI_Handler               	; 2 NMI Handler
        DCD     HardFault_Handler         	; 3 Hard Fault Handler
        DCD     MemManage_Handler         	; 4 MPU Fault Handler
        DCD     BusFault_Handler          	; 5 Bus Fault Handler
        DCD     UsageFault_Handler        	; 6 Usage Fault Handler
__vector_table_0x1c
        DCD     Sign_Value                	; 7 Reserved
        DCD     UnHandled_Vector           	; 8 Reserved
        DCD     UnHandled_Vector           	; 9 Reserved
        DCD     UnHandled_Vector          	; 10 Reserved
        DCD     SVC_Handler               	; 11 SVCall Handler
        DCD     DebugMon_Handler          	; 12 Debug Monitor Handler
        DCD     UnHandled_Vector          	; 13 Reserved
        DCD     OS_CPU_PendSVHandler
        DCD     OS_CPU_SysTickHandler

; External Interrupts				
				
        DCD     WDT_IRQHandler 			; 16 Watchdog Timer
        DCD     TIMER0_IRQHandler		; 17 Timer0
        DCD     TIMER1_IRQHandler		; 18 Timer1
        DCD     TIMER2_IRQHandler		; 19 Timer2
        DCD     TIMER3_IRQHandler		; 20 Timer3
        DCD     UART0_IRQHandler		; 21 UART0
        DCD     UART1_IRQHandler		; 22 UART1
        DCD     UART2_IRQHandler		; 23 UART2
        DCD     UART3_IRQHandler		; 24 UART3
        DCD     PWM1_IRQHandler			; 25 PWM1
        DCD     I2C0_IRQHandler        		; 26 I2C0
        DCD     I2C1_IRQHandler        		; 27 I2C1
        DCD     I2C2_IRQHandler			; 28 I2C2
        DCD     SPI_IRQHandler     		; 29 SPI (only on the 175x/6x, reserved on other devices)
        DCD     SSP0_IRQHandler        		; 30 SSP0
        DCD     SSP1_IRQHandler        		; 31 SSP1
        DCD     PLL0_IRQHandler		    	; 32 PLL0 Lock (Main PLL)
        DCD     RTC_IRQHandler        		; 33 RTC
        DCD     EINT0_IRQHandler		; 34 External Interrupt 0
        DCD     EINT1_IRQHandler		; 35 External Interrupt 1
        DCD     EINT2_IRQHandler		; 36 External Interrupt 2
        DCD     EINT3_IRQHandler		; 37 External Interrupt 3
        DCD     ADC_IRQHandler        		; 38 A/D Converter
        DCD     BOD_IRQHandler        		; 39 Brown-Out Detect
        DCD     USB_IRQHandler			; 40 USB
        DCD     CAN_IRQHandler   		; 41 CAN
        DCD     DMA_IRQHandler			; 42 General Purpose DMA
        DCD     I2S_IRQHandler			; 43 I2S
        DCD     ETH_IRQHandler        		; 44 Ethernet
        DCD     SDIO_IRQHandler        		; 45 SD/MMC card I/F
        DCD     MCPWM_IRQHandler		; 46 Motor Control PWM
        DCD     QEI_IRQHandler			; 47 QEI
        DCD     PLL1_IRQHandler		    	; 48 PLL1 Lock (USB PLL)
        DCD     USBActivity_IRQHandler		; 49 USB Activity interrupt to wakeup
        DCD     CANActivity_IRQHandler		; 50 CAN Activity interrupt to wakeup
        DCD     UART4_IRQHandler		; 51 UART4
        DCD     SSP2_IRQHandler			; 52 SSP2
        DCD     LCD_IRQHandler			; 53 LCD
        DCD     GPIO_IRQHandler			; 54 GPIO
        DCD     PWM0_IRQHandler        		; 55 PWM0
        DCD     EEPROM_IRQHandler		; 56 EEPROM				


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

	PUBWEAK UnHandled_Vector
        SECTION .text:CODE:REORDER(1)
UnHandled_Vector
        B UnHandled_Vector

Default_Handler:
        B Default_Handler

      PUBWEAK WDT_IRQHandler    
      PUBWEAK TIMER0_IRQHandler   
      PUBWEAK TIMER1_IRQHandler   
      PUBWEAK TIMER2_IRQHandler   
      PUBWEAK TIMER3_IRQHandler   
      PUBWEAK UART0_IRQHandler  
      PUBWEAK UART1_IRQHandler  
      PUBWEAK UART2_IRQHandler  
      PUBWEAK UART3_IRQHandler  
      PUBWEAK PWM1_IRQHandler   
      PUBWEAK I2C0_IRQHandler   
      PUBWEAK I2C1_IRQHandler   
      PUBWEAK I2C2_IRQHandler   
      PUBWEAK SPI_IRQHandler    
      PUBWEAK SSP0_IRQHandler   
      PUBWEAK SSP1_IRQHandler   
      PUBWEAK PLL0_IRQHandler   
      PUBWEAK RTC_IRQHandler    
      PUBWEAK EINT0_IRQHandler  
      PUBWEAK EINT1_IRQHandler  
      PUBWEAK EINT2_IRQHandler  
      PUBWEAK EINT3_IRQHandler  
      PUBWEAK ADC_IRQHandler    
      PUBWEAK BOD_IRQHandler    
      PUBWEAK USB_IRQHandler    
      PUBWEAK CAN_IRQHandler    
      PUBWEAK DMA_IRQHandler  
      PUBWEAK I2S_IRQHandler    
      PUBWEAK ETH_IRQHandler         
      PUBWEAK SDIO_IRQHandler              
      PUBWEAK MCPWM_IRQHandler  
      PUBWEAK QEI_IRQHandler               
      PUBWEAK PLL1_IRQHandler             
      PUBWEAK USBActivity_IRQHandler       
      PUBWEAK CANActivity_IRQHandler       
      PUBWEAK UART4_IRQHandler         
      PUBWEAK SSP2_IRQHandler           
      PUBWEAK LCD_IRQHandler           
      PUBWEAK GPIO_IRQHandler           
      PUBWEAK PWM0_IRQHandler           
      PUBWEAK EEPROM_IRQHandler

WDT_IRQHandler
TIMER0_IRQHandler
TIMER1_IRQHandler
TIMER2_IRQHandler
TIMER3_IRQHandler
UART0_IRQHandler
UART1_IRQHandler
UART2_IRQHandler
UART3_IRQHandler
PWM1_IRQHandler
I2C0_IRQHandler
I2C1_IRQHandler
I2C2_IRQHandler
SPI_IRQHandler
SSP0_IRQHandler
SSP1_IRQHandler
PLL0_IRQHandler
RTC_IRQHandler
EINT0_IRQHandler
EINT1_IRQHandler
EINT2_IRQHandler
EINT3_IRQHandler
ADC_IRQHandler
BOD_IRQHandler
USB_IRQHandler
CAN_IRQHandler
DMA_IRQHandler
I2S_IRQHandler
ETH_IRQHandler
SDIO_IRQHandler          
MCPWM_IRQHandler
QEI_IRQHandler
PLL1_IRQHandler
USBActivity_IRQHandler
CANActivity_IRQHandler
UART4_IRQHandler
SSP2_IRQHandler
LCD_IRQHandler
GPIO_IRQHandler
PWM0_IRQHandler
EEPROM_IRQHandler

        END
