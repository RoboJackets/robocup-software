/***********************************************************************/
/*  This file is part of the uVision/ARM development tools             */
/*  Copyright KEIL ELEKTRONIK GmbH 2002-2005                           */
/***********************************************************************/
/*                                                                     */
/*  LPC2103.H:  Header file for Philips LPC2101 / LPC2102 / LPC2103    */
/*                                                                     */
/***********************************************************************/

#ifndef __LPC2103_H
#define __LPC2103_H

/* Vectored Interrupt Controller (VIC) */
#define VICIRQStatus   (*((volatile unsigned long *) 0xFFFFF000))
#define VICFIQStatus   (*((volatile unsigned long *) 0xFFFFF004))
#define VICRawIntr     (*((volatile unsigned long *) 0xFFFFF008))
#define VICIntSelect   (*((volatile unsigned long *) 0xFFFFF00C))
#define VICIntEnable   (*((volatile unsigned long *) 0xFFFFF010))
#define VICIntEnClr    (*((volatile unsigned long *) 0xFFFFF014))
#define VICSoftInt     (*((volatile unsigned long *) 0xFFFFF018))
#define VICSoftIntClr  (*((volatile unsigned long *) 0xFFFFF01C))
#define VICProtection  (*((volatile unsigned long *) 0xFFFFF020))
#define VICVectAddr    (*((volatile unsigned long *) 0xFFFFF030))
#define VICDefVectAddr (*((volatile unsigned long *) 0xFFFFF034))
#define VICVectAddr0   (*((volatile unsigned long *) 0xFFFFF100))
#define VICVectAddr1   (*((volatile unsigned long *) 0xFFFFF104))
#define VICVectAddr2   (*((volatile unsigned long *) 0xFFFFF108))
#define VICVectAddr3   (*((volatile unsigned long *) 0xFFFFF10C))
#define VICVectAddr4   (*((volatile unsigned long *) 0xFFFFF110))
#define VICVectAddr5   (*((volatile unsigned long *) 0xFFFFF114))
#define VICVectAddr6   (*((volatile unsigned long *) 0xFFFFF118))
#define VICVectAddr7   (*((volatile unsigned long *) 0xFFFFF11C))
#define VICVectAddr8   (*((volatile unsigned long *) 0xFFFFF120))
#define VICVectAddr9   (*((volatile unsigned long *) 0xFFFFF124))
#define VICVectAddr10  (*((volatile unsigned long *) 0xFFFFF128))
#define VICVectAddr11  (*((volatile unsigned long *) 0xFFFFF12C))
#define VICVectAddr12  (*((volatile unsigned long *) 0xFFFFF130))
#define VICVectAddr13  (*((volatile unsigned long *) 0xFFFFF134))
#define VICVectAddr14  (*((volatile unsigned long *) 0xFFFFF138))
#define VICVectAddr15  (*((volatile unsigned long *) 0xFFFFF13C))
#define VICVectCntl0   (*((volatile unsigned long *) 0xFFFFF200))
#define VICVectCntl1   (*((volatile unsigned long *) 0xFFFFF204))
#define VICVectCntl2   (*((volatile unsigned long *) 0xFFFFF208))
#define VICVectCntl3   (*((volatile unsigned long *) 0xFFFFF20C))
#define VICVectCntl4   (*((volatile unsigned long *) 0xFFFFF210))
#define VICVectCntl5   (*((volatile unsigned long *) 0xFFFFF214))
#define VICVectCntl6   (*((volatile unsigned long *) 0xFFFFF218))
#define VICVectCntl7   (*((volatile unsigned long *) 0xFFFFF21C))
#define VICVectCntl8   (*((volatile unsigned long *) 0xFFFFF220))
#define VICVectCntl9   (*((volatile unsigned long *) 0xFFFFF224))
#define VICVectCntl10  (*((volatile unsigned long *) 0xFFFFF228))
#define VICVectCntl11  (*((volatile unsigned long *) 0xFFFFF22C))
#define VICVectCntl12  (*((volatile unsigned long *) 0xFFFFF230))
#define VICVectCntl13  (*((volatile unsigned long *) 0xFFFFF234))
#define VICVectCntl14  (*((volatile unsigned long *) 0xFFFFF238))
#define VICVectCntl15  (*((volatile unsigned long *) 0xFFFFF23C))

/* Pin Connect Block */
#define PINSEL0        (*((volatile unsigned long *) 0xE002C000))
#define PINSEL1        (*((volatile unsigned long *) 0xE002C004))

/* General Purpose Input/Output (GPIO) */
#define IOPIN          (*((volatile unsigned long *) 0xE0028000))
#define IOSET          (*((volatile unsigned long *) 0xE0028004))
#define IODIR          (*((volatile unsigned long *) 0xE0028008))
#define IOCLR          (*((volatile unsigned long *) 0xE002800C))

/* Fast General Purpose Input/Output (GPIO) */
#define FIODIR         (*((volatile unsigned long *) 0x3FFFC000))
#define FIOMASK        (*((volatile unsigned long *) 0x3FFFC010))
#define FIOPIN         (*((volatile unsigned long *) 0x3FFFC014))
#define FIOSET         (*((volatile unsigned long *) 0x3FFFC018))
#define FIOCLR         (*((volatile unsigned long *) 0x3FFFC01C))

/* Memory Accelerator Module (MAM) */
#define MAMCR          (*((volatile unsigned char *) 0xE01FC000))
#define MAMTIM         (*((volatile unsigned char *) 0xE01FC004))
#define MEMMAP         (*((volatile unsigned char *) 0xE01FC040))

/* Phase Locked Loop (PLL) */
#define PLLCON         (*((volatile unsigned char *) 0xE01FC080))
#define PLLCFG         (*((volatile unsigned char *) 0xE01FC084))
#define PLLSTAT        (*((volatile unsigned short*) 0xE01FC088))
#define PLLFEED        (*((volatile unsigned char *) 0xE01FC08C))

/* APB Divider */
#define APBDIV         (*((volatile unsigned char *) 0xE01FC100))

/* Power Control */
#define PCON           (*((volatile unsigned char *) 0xE01FC0C0))
#define PCONP          (*((volatile unsigned long *) 0xE01FC0C4))

/* External Interrupts */
#define EXTINT         (*((volatile unsigned char *) 0xE01FC140))
#define EXTWAKE        (*((volatile unsigned char *) 0xE01FC144))
#define EXTMODE        (*((volatile unsigned char *) 0xE01FC148))
#define EXTPOLAR       (*((volatile unsigned char *) 0xE01FC14C))

/* Timer 0 */
#define T0IR           (*((volatile unsigned char *) 0xE0004000))
#define T0TCR          (*((volatile unsigned char *) 0xE0004004))
#define T0TC           (*((volatile unsigned long *) 0xE0004008))
#define T0PR           (*((volatile unsigned long *) 0xE000400C))
#define T0PC           (*((volatile unsigned long *) 0xE0004010))
#define T0MCR          (*((volatile unsigned short*) 0xE0004014))
#define T0MR0          (*((volatile unsigned long *) 0xE0004018))
#define T0MR1          (*((volatile unsigned long *) 0xE000401C))
#define T0MR2          (*((volatile unsigned long *) 0xE0004020))
#define T0MR3          (*((volatile unsigned long *) 0xE0004024))
#define T0CCR          (*((volatile unsigned short*) 0xE0004028))
#define T0CR0          (*((volatile unsigned long *) 0xE000402C))
#define T0CR1          (*((volatile unsigned long *) 0xE0004030))
#define T0CR2          (*((volatile unsigned long *) 0xE0004034))
#define T0CR3          (*((volatile unsigned long *) 0xE0004038))
#define T0EMR          (*((volatile unsigned short*) 0xE000403C))
#define T0CTCR         (*((volatile unsigned char *) 0xE0004070))
#define T0PWMCON       (*((volatile unsigned long *) 0xE0004074))

/* Timer 1 */
#define T1IR           (*((volatile unsigned char *) 0xE0008000))
#define T1TCR          (*((volatile unsigned char *) 0xE0008004))
#define T1TC           (*((volatile unsigned long *) 0xE0008008))
#define T1PR           (*((volatile unsigned long *) 0xE000800C))
#define T1PC           (*((volatile unsigned long *) 0xE0008010))
#define T1MCR          (*((volatile unsigned short*) 0xE0008014))
#define T1MR0          (*((volatile unsigned long *) 0xE0008018))
#define T1MR1          (*((volatile unsigned long *) 0xE000801C))
#define T1MR2          (*((volatile unsigned long *) 0xE0008020))
#define T1MR3          (*((volatile unsigned long *) 0xE0008024))
#define T1CCR          (*((volatile unsigned short*) 0xE0008028))
#define T1CR0          (*((volatile unsigned long *) 0xE000802C))
#define T1CR1          (*((volatile unsigned long *) 0xE0008030))
#define T1CR2          (*((volatile unsigned long *) 0xE0008034))
#define T1CR3          (*((volatile unsigned long *) 0xE0008038))
#define T1EMR          (*((volatile unsigned short*) 0xE000803C))
#define T1CTCR         (*((volatile unsigned char *) 0xE0008070))
#define T1PWMCON       (*((volatile unsigned long *) 0xE0008074))

/* Universal Asynchronous Receiver Transmitter 0 (UART0) */
#define U0RBR          (*((volatile unsigned char *) 0xE000C000))
#define U0THR          (*((volatile unsigned char *) 0xE000C000))
#define U0IER          (*((volatile unsigned long *) 0xE000C004))
#define U0IIR          (*((volatile unsigned long *) 0xE000C008))
#define U0FCR          (*((volatile unsigned char *) 0xE000C008))
#define U0LCR          (*((volatile unsigned char *) 0xE000C00C))
#define U0LSR          (*((volatile unsigned char *) 0xE000C014))
#define U0SCR          (*((volatile unsigned char *) 0xE000C01C))
#define U0DLL          (*((volatile unsigned char *) 0xE000C000))
#define U0DLM          (*((volatile unsigned char *) 0xE000C004))
#define U0ACR          (*((volatile unsigned long *) 0xE000C020))
#define U0FDR          (*((volatile unsigned long *) 0xE000C028))
#define U0TER          (*((volatile unsigned char *) 0xE000C030))

/* Universal Asynchronous Receiver Transmitter 1 (UART1) */
#define U1RBR          (*((volatile unsigned char *) 0xE0010000))
#define U1THR          (*((volatile unsigned char *) 0xE0010000))
#define U1IER          (*((volatile unsigned long *) 0xE0010004))
#define U1IIR          (*((volatile unsigned long *) 0xE0010008))
#define U1FCR          (*((volatile unsigned char *) 0xE0010008))
#define U1LCR          (*((volatile unsigned char *) 0xE001000C))
#define U1MCR          (*((volatile unsigned char *) 0xE0010010))
#define U1LSR          (*((volatile unsigned char *) 0xE0010014))
#define U1MSR          (*((volatile unsigned char *) 0xE0010018))
#define U1SCR          (*((volatile unsigned char *) 0xE001001C))
#define U1DLL          (*((volatile unsigned char *) 0xE0010000))
#define U1DLM          (*((volatile unsigned char *) 0xE0010004))
#define U1ACR          (*((volatile unsigned long *) 0xE0010020))
#define U1FDR          (*((volatile unsigned long *) 0xE0010028))
#define U1TER          (*((volatile unsigned char *) 0xE0010030))

/* Inter-Integrated Circuit interface 0 (I2C0) */
#define I2C0CONSET     (*((volatile unsigned char *) 0xE001C000))
#define I2C0STAT       (*((volatile unsigned char *) 0xE001C004))
#define I2C0DAT        (*((volatile unsigned char *) 0xE001C008))
#define I2C0ADR        (*((volatile unsigned char *) 0xE001C00C))
#define I2C0SCLH       (*((volatile unsigned short*) 0xE001C010))
#define I2C0SCLL       (*((volatile unsigned short*) 0xE001C014))
#define I2C0CONCLR     (*((volatile unsigned char *) 0xE001C018))

/* Serial Peripheral Interface 0 (SPI0) */
#define S0SPCR         (*((volatile unsigned short*) 0xE0020000))
#define S0SPSR         (*((volatile unsigned char *) 0xE0020004))
#define S0SPDR         (*((volatile unsigned short*) 0xE0020008))
#define S0SPCCR        (*((volatile unsigned char *) 0xE002000C))
#define S0SPINT        (*((volatile unsigned char *) 0xE002001C))

/* Real Time Clock (RTC) */
#define ILR            (*((volatile unsigned char *) 0xE0024000))
#define CTC            (*((volatile unsigned short*) 0xE0024004))
#define CCR            (*((volatile unsigned char *) 0xE0024008))
#define CIIR           (*((volatile unsigned char *) 0xE002400C))
#define AMR            (*((volatile unsigned char *) 0xE0024010))
#define CTIME0         (*((volatile unsigned long *) 0xE0024014))
#define CTIME1         (*((volatile unsigned long *) 0xE0024018))
#define CTIME2         (*((volatile unsigned long *) 0xE002401C))
#define SEC            (*((volatile unsigned char *) 0xE0024020))
#define MIN            (*((volatile unsigned char *) 0xE0024024))
#define HOUR           (*((volatile unsigned char *) 0xE0024028))
#define DOM            (*((volatile unsigned char *) 0xE002402C))
#define DOW            (*((volatile unsigned char *) 0xE0024030))
#define DOY            (*((volatile unsigned short*) 0xE0024034))
#define MONTH          (*((volatile unsigned char *) 0xE0024038))
#define YEAR           (*((volatile unsigned short*) 0xE002403C))
#define ALSEC          (*((volatile unsigned char *) 0xE0024060))
#define ALMIN          (*((volatile unsigned char *) 0xE0024064))
#define ALHOUR         (*((volatile unsigned char *) 0xE0024068))
#define ALDOM          (*((volatile unsigned char *) 0xE002406C))
#define ALDOW          (*((volatile unsigned char *) 0xE0024070))
#define ALDOY          (*((volatile unsigned short*) 0xE0024074))
#define ALMON          (*((volatile unsigned char *) 0xE0024078))
#define ALYEAR         (*((volatile unsigned short*) 0xE002407C))
#define PREINT         (*((volatile unsigned short*) 0xE0024080))
#define PREFRAC        (*((volatile unsigned short*) 0xE0024084))

/* Analog/Digital Converter (ADC) */
#define ADCR           (*((volatile unsigned long *) 0xE0034000))
#define ADGDR          (*((volatile unsigned long *) 0xE0034004))
#define ADINTEN        (*((volatile unsigned long *) 0xE003400C))
#define ADDR0          (*((volatile unsigned long *) 0xE0034010))
#define ADDR1          (*((volatile unsigned long *) 0xE0034014))
#define ADDR2          (*((volatile unsigned long *) 0xE0034018))
#define ADDR3          (*((volatile unsigned long *) 0xE003401C))
#define ADDR4          (*((volatile unsigned long *) 0xE0034020))
#define ADDR5          (*((volatile unsigned long *) 0xE0034024))
#define ADDR6          (*((volatile unsigned long *) 0xE0034028))
#define ADDR7          (*((volatile unsigned long *) 0xE003402C))
#define ADSTAT         (*((volatile unsigned long *) 0xE0034030))

/* Inter-Integrated Circuit interface 1 (I2C1) */
#define I2C1CONSET     (*((volatile unsigned char *) 0xE005C000))
#define I2C1STAT       (*((volatile unsigned char *) 0xE005C004))
#define I2C1DAT        (*((volatile unsigned char *) 0xE005C008))
#define I2C1ADR        (*((volatile unsigned char *) 0xE005C00C))
#define I2C1SCLH       (*((volatile unsigned short*) 0xE005C010))
#define I2C1SCLL       (*((volatile unsigned short*) 0xE005C014))
#define I2C1CONCLR     (*((volatile unsigned char *) 0xE005C018))

/* Synchronous Serial Port interface (SSP) */
#define SSPCR0         (*((volatile unsigned short*) 0xE0068000))
#define SSPCR1         (*((volatile unsigned char *) 0xE0068004))
#define SSPDR          (*((volatile unsigned short*) 0xE0068008))
#define SSPSR          (*((volatile unsigned char *) 0xE006800C))
#define SSPCPSR        (*((volatile unsigned char *) 0xE0068010))
#define SSPIMSC        (*((volatile unsigned char *) 0xE0068014))
#define SSPRIS         (*((volatile unsigned char *) 0xE0068018))
#define SSPMIS         (*((volatile unsigned char *) 0xE006801C))
#define SSPICR         (*((volatile unsigned char *) 0xE0068020))

/* Timer 2 */
#define T2IR           (*((volatile unsigned char *) 0xE0070000))
#define T2TCR          (*((volatile unsigned char *) 0xE0070004))
#define T2TC           (*((volatile unsigned long *) 0xE0070008))
#define T2PR           (*((volatile unsigned long *) 0xE007000C))
#define T2PC           (*((volatile unsigned long *) 0xE0070010))
#define T2MCR          (*((volatile unsigned short*) 0xE0070014))
#define T2MR0          (*((volatile unsigned long *) 0xE0070018))
#define T2MR1          (*((volatile unsigned long *) 0xE007001C))
#define T2MR2          (*((volatile unsigned long *) 0xE0070020))
#define T2MR3          (*((volatile unsigned long *) 0xE0070024))
#define T2CCR          (*((volatile unsigned short*) 0xE0070028))
#define T2CR0          (*((volatile unsigned long *) 0xE007002C))
#define T2CR1          (*((volatile unsigned long *) 0xE0070030))
#define T2CR2          (*((volatile unsigned long *) 0xE0070034))
#define T2EMR          (*((volatile unsigned short*) 0xE007003C))
#define T2CTCR         (*((volatile unsigned char *) 0xE0070070))
#define T2PWMCON       (*((volatile unsigned long *) 0xE0070074))

/* Timer 3 */
#define T3IR           (*((volatile unsigned char *) 0xE0074000))
#define T3TCR          (*((volatile unsigned char *) 0xE0074004))
#define T3TC           (*((volatile unsigned long *) 0xE0074008))
#define T3PR           (*((volatile unsigned long *) 0xE007400C))
#define T3PC           (*((volatile unsigned long *) 0xE0074010))
#define T3MCR          (*((volatile unsigned short*) 0xE0074014))
#define T3MR0          (*((volatile unsigned long *) 0xE0074018))
#define T3MR4          (*((volatile unsigned long *) 0xE007401C))
#define T3MR2          (*((volatile unsigned long *) 0xE0074020))
#define T3MR3          (*((volatile unsigned long *) 0xE0074024))
#define T3CCR          (*((volatile unsigned short*) 0xE0074028))
#define T3CR0          (*((volatile unsigned long *) 0xE007402C))
#define T3CR1          (*((volatile unsigned long *) 0xE0074030))
#define T3CR2          (*((volatile unsigned long *) 0xE0074034))
#define T3EMR          (*((volatile unsigned short*) 0xE007403C))
#define T3CTCR         (*((volatile unsigned char *) 0xE0074070))
#define T3PWMCON       (*((volatile unsigned long *) 0xE0074074))

/* Reset Source Identification */
#define RSIR           (*((volatile unsigned char *) 0xE01FC180))

/* Code Security Protection */
#define CPSR           (*((volatile unsigned long *) 0xE01FC184))

/* Syscon Miscellaneous */
#define SCS            (*((volatile unsigned long *) 0xE01FC1A0))

/* Watchdog timer */
#define WDMOD          (*((volatile unsigned char *) 0xE0000000))
#define WDTC           (*((volatile unsigned long *) 0xE0000004))
#define WDFEED         (*((volatile unsigned char *) 0xE0000008))
#define WDTV           (*((volatile unsigned long *) 0xE000000C))

#endif  // __LPC2103_H
