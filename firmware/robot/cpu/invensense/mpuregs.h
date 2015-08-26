/*******************************************************************************
 *
 * $Id: mpuregs.h 2079 2010-04-09 01:03:05Z nroyer $
 *
 *******************************************************************************/

/*******************************************************************************
 * Copyright (c) 2009 InvenSense Corporation, All Rights Reserved.
 *******************************************************************************/

/*******************************************************************************

    File Name:    MPURegs.h

    Description:  Contains register definitions for InvenSense MPU-3000.

********************************************************************************/

#ifndef MPUREGS_H
#define MPUREGS_H

/*==== MPU REGISTER SET ====*/

typedef enum {
    MPUREG_WHO_AM_I = 0,     /* 00 0x00 */
    MPUREG_PRODUCT_ID,       /* 01 0x01 */
    MPUREG_02_RSVD,          /* 02 0x02 */
    MPUREG_03_RSVD,          /* 03 0x03 */
    MPUREG_04_RSVD,          /* 04 0x04 */
    MPUREG_05_RSVD,          /* 05 0x05 */
    MPUREG_06_RSVD,          /* 06 0x06 */
    MPUREG_07_RSVD,          /* 07 0x07 */
    MPUREG_08_RSVD,          /* 08 0x08 */
    MPUREG_09_RSVD,          /* 09 0x09 */
    MPUREG_0A_RSVD,          /* 10 0x0a */
    MPUREG_0B_RSVD,          /* 11 0x0b */
    MPUREG_X_OFFS_USRH,      /* 12 0x0c */
    MPUREG_X_OFFS_USRL,      /* 13 0x0d */
    MPUREG_Y_OFFS_USRH,      /* 14 0x0e */
    MPUREG_Y_OFFS_USRL,      /* 15 0x0f */
    MPUREG_Z_OFFS_USRH,      /* 16 0x10 */
    MPUREG_Z_OFFS_USRL,      /* 17 0x11 */
    MPUREG_FIFO_EN1,         /* 18 0x12 */
    MPUREG_FIFO_EN2,         /* 19 0x13 */
    MPUREG_AUX_SLV_ADDR,     /* 20 0x14 */
    MPUREG_SMPLRT_DIV,       /* 21 0x15 */
    MPUREG_DLPF_FS_SYNC,     /* 22 0x16 */
    MPUREG_INT_CFG,          /* 23 0x17 */
    MPUREG_ACCEL_BURST_ADDR, /* 24 0x18 */
    MPUREG_19_RSVD,          /* 25 0x19 */
    MPUREG_INT_STATUS,       /* 26 0x1a */
    MPUREG_TEMP_OUT_H,       /* 27 0x1b */
    MPUREG_TEMP_OUT_L,       /* 28 0x1c */
    MPUREG_GYRO_XOUT_H,      /* 29 0x1d */
    MPUREG_GYRO_XOUT_L,      /* 30 0x1e */
    MPUREG_GYRO_YOUT_H,      /* 31 0x1f */
    MPUREG_GYRO_YOUT_L,      /* 32 0x20 */
    MPUREG_GYRO_ZOUT_H,      /* 33 0x21 */
    MPUREG_GYRO_ZOUT_L,      /* 34 0x22 */
    MPUREG_23_RSVD,          /* 35 0x23 */
    MPUREG_24_RSVD,          /* 36 0x24 */
    MPUREG_25_RSVD,          /* 37 0x25 */
    MPUREG_26_RSVD,          /* 38 0x26 */
    MPUREG_27_RSVD,          /* 39 0x27 */
    MPUREG_28_RSVD,          /* 40 0x28 */
    MPUREG_29_RSVD,          /* 41 0x29 */
    MPUREG_2A_RSVD,          /* 42 0x2a */
    MPUREG_2B_RSVD,          /* 43 0x2b */
    MPUREG_2C_RSVD,          /* 44 0x2c */
    MPUREG_2D_RSVD,          /* 45 0x2d */
    MPUREG_2E_RSVD,          /* 46 0x2e */
    MPUREG_2F_RSVD,          /* 47 0x2f */
    MPUREG_30_RSVD,          /* 48 0x30 */
    MPUREG_31_RSVD,          /* 49 0x31 */
    MPUREG_32_RSVD,          /* 50 0x32 */
    MPUREG_33_RSVD,          /* 51 0x33 */
    MPUREG_34_RSVD,          /* 52 0x34 */
    MPUREG_DMP_CFG_1,        /* 53 0x35 */
    MPUREG_DMP_CFG_2,        /* 54 0x36 */
    MPUREG_BANK_SEL,         /* 55 0x37 */
    MPUREG_MEM_START_ADDR,   /* 56 0x38 */
    MPUREG_MEM_R_W,          /* 57 0x39 */
    MPUREG_FIFO_COUNTH,      /* 58 0x3a */
    MPUREG_FIFO_COUNTL,      /* 59 0x3b */
    MPUREG_FIFO_R_W,         /* 60 0x3c */
    MPUREG_USER_CTRL,        /* 61 0x3d */
    MPUREG_PWR_MGM,          /* 62 0x3e */
    MPUREG_3F_RSVD,          /* 63 0x3f */
    NUM_OF_MPU_REGISTERS     /* 64 0x40 */

} tMPUUserRegisters,
    MPU_USER_REGISTERS;

/*==== BITS FOR MPU ====*/

/*---- MPU 'FIFO_EN1' register (12) ----*/
#define BIT_TEMP_OUT 0x80
#define BIT_GYRO_XOUT 0x40
#define BIT_GYRO_YOUT 0x20
#define BIT_GYRO_ZOUT 0x10
#define BIT_ACCEL_XOUT 0x08
#define BIT_ACCEL_YOUT 0x04
#define BIT_ACCEL_ZOUT 0x02
#define BIT_AUX_1OUT 0x01
/*---- MPU 'FIFO_EN2' register (13) ----*/
#define BIT_AUX_2OUT 0x02
#define BIT_AUX_3OUT 0x01
/*---- MPU 'DLPF_FS_SYNC' register (16) ----*/
#define BITS_EXT_SYNC_SET 0xE0
#define BITS_FS_SEL 0x18
#define BITS_DLPF_CFG 0x07
/*---- MPU 'INT_CFG' register (17) ----*/
#define BIT_ACTL 0x80
#define BIT_OPEN 0x40
#define BIT_LATCH_INT_EN 0x20
#define BIT_INT_ANYRD_2CLEAR 0x10
#define BIT_MPU_RDY_EN 0x04
#define BIT_DMP_INT_EN 0x02
#define BIT_RAW_RDY_EN 0x01
/*---- MPU 'INT_STATUS' register (1A) ----*/
#define BIT_MPU_RDY 0x04
#define BIT_DMP_INT 0x02
#define BIT_RAW_RDY 0x01
/*---- MPU 'BANK_SEL' register (37) ----*/
#define BITS_MEM_SEL 0x0f
/*---- MPU 'USER_CTRL' register (3D) ----*/
#define BIT_DMP_EN 0x80
#define BIT_FIFO_EN 0x40
#define BIT_AUX_IF_EN 0x20
#define BIT_AUX_RD_LENG 0x10
#define BIT_AUX_IF_RST 0x08
#define BIT_DMP_RST 0x04
#define BIT_FIFO_RST 0x02
#define BIT_GYRO_RST 0x01
/*---- MPU 'PWR_MGM' register (3E) ----*/
#define BIT_H_RESET 0x80
#define BIT_SLEEP 0x40
#define BIT_STBY_XG 0x20
#define BIT_STBY_YG 0x10
#define BIT_STBY_ZG 0x08
#define BITS_CLKSEL 0x07

/*---- MPU Chip Version ----*/
#define MPUID_VERSIONB1 7  // MPU-3100 (B1) Device

typedef enum {
    MPUMEM_RAM_BANK_0 = 0,
    MPUMEM_RAM_BANK_1,
    MPUMEM_RAM_BANK_2,
    MPUMEM_RAM_BANK_3

} tMPUMemoryBanks,
    MPU_MEMORY_BANKS;

#endif
