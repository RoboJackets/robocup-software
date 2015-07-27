;/*
; * @brief LPC17xx/40xx startup code
; *
; * @note
; * Copyright(C) NXP Semiconductors, 2012
; * All rights reserved.
; *
; * @par
; * Software that is described herein is for illustrative purposes only
; * which provides customers with programming information regarding the
; * LPC products.  This software is supplied "AS IS" without any warranties of
; * any kind, and NXP Semiconductors and its licensor disclaim any and
; * all warranties, express or implied, including all implied warranties of
; * merchantability, fitness for a particular purpose and non-infringement of
; * intellectual property rights.  NXP Semiconductors assumes no responsibility
; * or liability for the use of the software, conveys no license or rights under any
; * patent, copyright, mask work right, or any other intellectual property rights in
; * or to any products. NXP Semiconductors reserves the right to make changes
; * in the software without notification. NXP Semiconductors also makes no
; * representation or warranty that such application will be suitable for the
; * specified use without further testing or modification.
; *
; * @par
; * Permission to use, copy, modify, and distribute this software and its
; * documentation is hereby granted, under NXP Semiconductors' and its
; * licensor's relevant copyrights in the software, without fee, provided that it
; * is used in conjunction with NXP Semiconductors microcontrollers.  This
; * copyright, permission, and disclaimer notice must appear in all copies of
; * this code.
; */

#include "sys_config.h"

; <h> Stack Configuration
;   <o> Stack Size (in Bytes) <0x0-0xFFFFFFFF:8>
; </h>

Stack_Size      EQU     0x00000200

                AREA    STACK, NOINIT, READWRITE, ALIGN=3
Stack_Mem       SPACE   Stack_Size
__initial_sp

; <h> Heap Configuration
;   <o>  Heap Size (in Bytes) <0x0-0xFFFFFFFF:8>
; </h>

Heap_Size       EQU     0x4000

                AREA    HEAP, NOINIT, READWRITE, ALIGN=3
__heap_base
Heap_Mem        SPACE   Heap_Size
__heap_limit

				EXTERN xPortSysTickHandler
				EXTERN xPortPendSVHandler
				EXTERN vPortSVCHandler

                PRESERVE8
                THUMB

; Vector Table Mapped to Address 0 at Reset

                AREA    RESET, DATA, READONLY
                EXPORT  __Vectors

Sign_Value		EQU		0x5A5A5A5A

__Vectors       DCD     __initial_sp              	; 0 Top of Stack
                DCD     Reset_Handler             	; 1 Reset Handler
                DCD     NMI_Handler               	; 2 NMI Handler
                DCD     HardFault_Handler         	; 3 Hard Fault Handler
                DCD     MemManage_Handler         	; 4 MPU Fault Handler
                DCD     BusFault_Handler          	; 5 Bus Fault Handler
                DCD     UsageFault_Handler        	; 6 Usage Fault Handler
                DCD     Sign_Value                	; 7 Reserved
                DCD     UnHandled_Vector           	; 8 Reserved
                DCD     UnHandled_Vector           	; 9 Reserved
                DCD     UnHandled_Vector          	; 10 Reserved
                DCD     vPortSVCHandler            	; 11 FreeRTOS SVCall Handler
                DCD     DebugMon_Handler          	; 12 Debug Monitor Handler
                DCD     UnHandled_Vector          	; 13 Reserved
                DCD     xPortPendSVHandler         	; 14 FreeRTOS PendSV Handler
                DCD     xPortSysTickHandler        	; 15 FreeRTOS SysTick Handler

                ; External Interrupts				
				
				DCD		WDT_IRQHandler	 			; 16 Watchdog Timer
				DCD		TIMER0_IRQHandler			; 17 Timer0
				DCD		TIMER1_IRQHandler		    ; 18 Timer1
				DCD		TIMER2_IRQHandler			; 19 Timer2
				DCD		TIMER3_IRQHandler		    ; 20 Timer3
				DCD		UART0_IRQHandler			; 21 UART0
				DCD		UART1_IRQHandler			; 22 UART1
				DCD		UART2_IRQHandler			; 23 UART2
				DCD		UART3_IRQHandler			; 24 UART3
				DCD		PWM1_IRQHandler			    ; 25 PWM1
				DCD		I2C0_IRQHandler				; 26 I2C0
				DCD		I2C1_IRQHandler				; 27 I2C1
				DCD		I2C2_IRQHandler			    ; 28 I2C2
#ifndef CHIP_LPC175X_6X
				DCD		UnHandled_Vector           	        ; 29 Reserved
#else				
				DCD		SPI_IRQHandler     			; 29 SPI (only on the 175x/6x, reserved on other devices)
#endif				
				DCD		SSP0_IRQHandler				; 30 SSP0
				DCD		SSP1_IRQHandler				; 31 SSP1
				DCD		PLL0_IRQHandler		    	; 32 PLL0 Lock (Main PLL)
				DCD		RTC_IRQHandler				; 33 RTC
				DCD		EINT0_IRQHandler			; 34 External Interrupt 0
				DCD		EINT1_IRQHandler			; 35 External Interrupt 1
				DCD		EINT2_IRQHandler			; 36 External Interrupt 2
				DCD		EINT3_IRQHandler			; 37 External Interrupt 3
				DCD		ADC_IRQHandler				; 38 A/D Converter
				DCD		BOD_IRQHandler				; 39 Brown-Out Detect
				DCD		USB_IRQHandler			    ; 40 USB
				DCD		CAN_IRQHandler   			; 41 CAN
				DCD		DMA_IRQHandler			    ; 42 General Purpose DMA
				DCD		I2S_IRQHandler			    ; 43 I2S
				DCD		ETH_IRQHandler				; 44 Ethernet
#ifndef CHIP_LPC175X_6X				
				DCD		SDIO_IRQHandler				; 45 SD/MMC card I/F
#else
				DCD		RIT_IRQHandler				; 45 RITIMER
#endif				
				DCD		MCPWM_IRQHandler			; 46 Motor Control PWM
				DCD		QEI_IRQHandler			    ; 47 QEI
				DCD		PLL1_IRQHandler		    	; 48 PLL1 Lock (USB PLL)
				DCD		USBActivity_IRQHandler		; 49 USB Activity interrupt to wakeup
				DCD		CANActivity_IRQHandler		; 50 CAN Activity interrupt to wakeup
#ifndef CHIP_LPC175X_6X
				DCD		UART4_IRQHandler			; 51 UART4
				DCD		SSP2_IRQHandler			    ; 52 SSP2
				DCD		LCD_IRQHandler			    ; 53 LCD
				DCD		GPIO_IRQHandler			    ; 54 GPIO
				DCD		PWM0_IRQHandler				; 55 PWM0
				DCD		EEPROM_IRQHandler			; 56 EEPROM				
#endif

;                IF      :LNOT::DEF:NO_CRP
;                AREA    |.ARM.__at_0x02FC|, CODE, READONLY
;CRP_Key         DCD     0xFFFFFFFF
;                ENDIF

                AREA    |.text|, CODE, READONLY

; Reset Handler

Reset_Handler   PROC
                EXPORT  Reset_Handler             [WEAK]
                IMPORT  __main
				IMPORT  SystemInit
				LDR		R0, =SystemInit
				BLX		R0
                LDR     R0, =__main
                BX      R0
                ENDP

; Dummy Exception Handlers (infinite loops which can be modified)

NMI_Handler     PROC
                EXPORT  NMI_Handler               [WEAK]
                B       .
                ENDP
HardFault_Handler\
                PROC
                EXPORT  HardFault_Handler         [WEAK]
                B       .
                ENDP
MemManage_Handler\
                PROC
                EXPORT  MemManage_Handler         [WEAK]
                B       .
                ENDP
BusFault_Handler\
                PROC
                EXPORT  BusFault_Handler          [WEAK]
                B       .
                ENDP
UsageFault_Handler\
                PROC
                EXPORT  UsageFault_Handler        [WEAK]
                B       .
                ENDP
SVC_Handler     PROC
                EXPORT  SVC_Handler               [WEAK]
                B       .
                ENDP
DebugMon_Handler\
                PROC
                EXPORT  DebugMon_Handler          [WEAK]
                B       .
                ENDP
PendSV_Handler  PROC
                EXPORT  PendSV_Handler            [WEAK]
                B       .
                ENDP
SysTick_Handler PROC
                EXPORT  SysTick_Handler           [WEAK]
                B       .
                ENDP
UnHandled_Vector	PROC
                EXPORT  UnHandled_Vector          [WEAK]
                B       .
                ENDP

Default_Handler PROC

                EXPORT  WDT_IRQHandler            [WEAK]
				EXPORT  TIMER0_IRQHandler         [WEAK]
				EXPORT  TIMER1_IRQHandler         [WEAK]
				EXPORT  TIMER2_IRQHandler         [WEAK]
				EXPORT  TIMER3_IRQHandler         [WEAK]
				EXPORT  UART0_IRQHandler          [WEAK]
				EXPORT  UART1_IRQHandler          [WEAK]
				EXPORT  UART2_IRQHandler          [WEAK]
				EXPORT  UART3_IRQHandler          [WEAK]
				EXPORT  PWM1_IRQHandler           [WEAK]
				EXPORT  I2C0_IRQHandler           [WEAK]
				EXPORT  I2C1_IRQHandler           [WEAK]
				EXPORT  I2C2_IRQHandler           [WEAK]
				EXPORT  SPI_IRQHandler            [WEAK]
				EXPORT  SSP0_IRQHandler           [WEAK]
				EXPORT  SSP1_IRQHandler           [WEAK]
				EXPORT  PLL0_IRQHandler           [WEAK]
				EXPORT  RTC_IRQHandler            [WEAK]
				EXPORT  EINT0_IRQHandler          [WEAK]
				EXPORT  EINT1_IRQHandler          [WEAK]
				EXPORT  EINT2_IRQHandler          [WEAK]
				EXPORT  EINT3_IRQHandler          [WEAK]
				EXPORT  ADC_IRQHandler            [WEAK]
				EXPORT  BOD_IRQHandler            [WEAK]
				EXPORT  USB_IRQHandler            [WEAK]
				EXPORT  CAN_IRQHandler            [WEAK]
				EXPORT  DMA_IRQHandler            [WEAK]
				EXPORT  I2S_IRQHandler            [WEAK]
				EXPORT  ETH_IRQHandler            [WEAK]
#ifndef CHIP_LPC175X_6X				
				EXPORT  SDIO_IRQHandler            [WEAK]
#else
				EXPORT  RIT_IRQHandler            [WEAK]
#endif				
				EXPORT  MCPWM_IRQHandler          [WEAK]
				EXPORT  QEI_IRQHandler            [WEAK]
				EXPORT  PLL1_IRQHandler           [WEAK]
				EXPORT  USBActivity_IRQHandler    [WEAK]
				EXPORT  CANActivity_IRQHandler    [WEAK]
#ifndef CHIP_LPC175X_6X				
				EXPORT  UART4_IRQHandler          [WEAK]
				EXPORT  SSP2_IRQHandler           [WEAK]
				EXPORT  LCD_IRQHandler            [WEAK]
				EXPORT  GPIO_IRQHandler           [WEAK]
				EXPORT  PWM0_IRQHandler           [WEAK]
				EXPORT  EEPROM_IRQHandler         [WEAK]
#endif
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
#ifndef CHIP_LPC175X_6X
SDIO_IRQHandler        
#else
RIT_IRQHandler
#endif  
MCPWM_IRQHandler
QEI_IRQHandler
PLL1_IRQHandler
USBActivity_IRQHandler
CANActivity_IRQHandler
#ifndef CHIP_LPC175X_6X
UART4_IRQHandler
SSP2_IRQHandler
LCD_IRQHandler
GPIO_IRQHandler
PWM0_IRQHandler
EEPROM_IRQHandler
#endif

                B       .

                ENDP

                ALIGN

; User Initial Stack & Heap

                IF      :DEF:__MICROLIB

                EXPORT  __initial_sp
                EXPORT  __heap_base
                EXPORT  __heap_limit

                ELSE

                IMPORT  __use_two_region_memory
                EXPORT  __user_initial_stackheap
__user_initial_stackheap

                LDR     R0, =  Heap_Mem
                LDR     R1, =(Stack_Mem + Stack_Size)
                LDR     R2, = (Heap_Mem +  Heap_Size)
                LDR     R3, = Stack_Mem
                BX      LR

                ALIGN

                ENDIF

				END
				
