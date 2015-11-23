/*****************************************************************************/
//  @file   CC120x_spi.h  
//  
//  @brief  header file that defines a minimum set of neccessary functions
//          to communicate with CC120X over SPI as well as the regsister 
//          mapping. 
//				 
//  Copyright (C) 2013 Texas Instruments Incorporated - http://www.ti.com/
//
//  Redistribution and use in source and binary forms, with or without
//  modification, are permitted provided that the following conditions
//  are met:
//
//    Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//
//    Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//
//    Neither the name of Texas Instruments Incorporated nor the names of
//    its contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
//  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
//  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
//  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
//  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
//  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
//  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
//  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
//  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
//  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
//  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
/****************************************************************************/

#ifndef CC120X_SPI_H
#define CC120X_SPI_H

#ifdef __cplusplus
extern "C" {
#endif
/******************************************************************************
 * INCLUDES
 */
#include "hal_types.h"
#include "hal_spi_rf_trxeb.h"

/******************************************************************************
 * CONSTANTS
 */
/* configuration registers */
#define CC120X_IOCFG3                   0x0000
#define CC120X_IOCFG2                   0x0001
#define CC120X_IOCFG1                   0x0002
#define CC120X_IOCFG0                   0x0003
#define CC120X_SYNC3                    0x0004
#define CC120X_SYNC2                    0x0005
#define CC120X_SYNC1                    0x0006
#define CC120X_SYNC0                    0x0007
#define CC120X_SYNC_CFG1                0x0008
#define CC120X_SYNC_CFG0                0x0009
#define CC120X_DEVIATION_M              0x000A
#define CC120X_MODCFG_DEV_E             0x000B
#define CC120X_DCFILT_CFG               0x000C
#define CC120X_PREAMBLE_CFG1            0x000D
#define CC120X_PREAMBLE_CFG0            0x000E
#define CC120X_IQIC                     0x000F
#define CC120X_CHAN_BW                  0x0010
#define CC120X_MDMCFG1                  0x0011
#define CC120X_MDMCFG0                  0x0012
#define CC120X_SYMBOL_RATE2             0x0013
#define CC120X_SYMBOL_RATE1             0x0014
#define CC120X_SYMBOL_RATE0             0x0015
#define CC120X_AGC_REF                  0x0016
#define CC120X_AGC_CS_THR               0x0017
#define CC120X_AGC_GAIN_ADJUST          0x0018
#define CC120X_AGC_CFG3                 0x0019
#define CC120X_AGC_CFG2                 0x001A
#define CC120X_AGC_CFG1                 0x001B
#define CC120X_AGC_CFG0                 0x001C
#define CC120X_FIFO_CFG                 0x001D
#define CC120X_DEV_ADDR                 0x001E
#define CC120X_SETTLING_CFG             0x001F
#define CC120X_FS_CFG                   0x0020
#define CC120X_WOR_CFG1                 0x0021
#define CC120X_WOR_CFG0                 0x0022
#define CC120X_WOR_EVENT0_MSB           0x0023
#define CC120X_WOR_EVENT0_LSB           0x0024
#define CC120X_RXDCM_TIME               0x0025
#define CC120X_PKT_CFG2                 0x0026
#define CC120X_PKT_CFG1                 0x0027
#define CC120X_PKT_CFG0                 0x0028
#define CC120X_RFEND_CFG1               0x0029
#define CC120X_RFEND_CFG0               0x002A
#define CC120X_PA_CFG1                  0x002B
#define CC120X_PA_CFG0                  0x002C
#define CC120X_ASK_CFG                  0x002D
#define CC120X_PKT_LEN                  0x002E
  
/* Extended Configuration Registers */
#define CC120X_IF_MIX_CFG               0x2F00
#define CC120X_FREQOFF_CFG              0x2F01
#define CC120X_TOC_CFG                  0x2F02
#define CC120X_MARC_SPARE               0x2F03
#define CC120X_ECG_CFG                  0x2F04
#define CC120X_MDMCFG2                  0x2F05
#define CC120X_EXT_CTRL                 0x2F06
#define CC120X_RCCAL_FINE               0x2F07
#define CC120X_RCCAL_COARSE             0x2F08
#define CC120X_RCCAL_OFFSET             0x2F09
#define CC120X_FREQOFF1                 0x2F0A
#define CC120X_FREQOFF0                 0x2F0B
#define CC120X_FREQ2                    0x2F0C
#define CC120X_FREQ1                    0x2F0D
#define CC120X_FREQ0                    0x2F0E
#define CC120X_IF_ADC2                  0x2F0F
#define CC120X_IF_ADC1                  0x2F10
#define CC120X_IF_ADC0                  0x2F11
#define CC120X_FS_DIG1                  0x2F12
#define CC120X_FS_DIG0                  0x2F13
#define CC120X_FS_CAL3                  0x2F14
#define CC120X_FS_CAL2                  0x2F15
#define CC120X_FS_CAL1                  0x2F16
#define CC120X_FS_CAL0                  0x2F17
#define CC120X_FS_CHP                   0x2F18
#define CC120X_FS_DIVTWO                0x2F19
#define CC120X_FS_DSM1                  0x2F1A
#define CC120X_FS_DSM0                  0x2F1B
#define CC120X_FS_DVC1                  0x2F1C
#define CC120X_FS_DVC0                  0x2F1D
#define CC120X_FS_LBI                   0x2F1E
#define CC120X_FS_PFD                   0x2F1F
#define CC120X_FS_PRE                   0x2F20
#define CC120X_FS_REG_DIV_CML           0x2F21
#define CC120X_FS_SPARE                 0x2F22
#define CC120X_FS_VCO4                  0x2F23
#define CC120X_FS_VCO3                  0x2F24
#define CC120X_FS_VCO2                  0x2F25
#define CC120X_FS_VCO1                  0x2F26
#define CC120X_FS_VCO0                  0x2F27
#define CC120X_GBIAS6                   0x2F28
#define CC120X_GBIAS5                   0x2F29
#define CC120X_GBIAS4                   0x2F2A
#define CC120X_GBIAS3                   0x2F2B
#define CC120X_GBIAS2                   0x2F2C
#define CC120X_GBIAS1                   0x2F2D
#define CC120X_GBIAS0                   0x2F2E
#define CC120X_IFAMP                    0x2F2F
#define CC120X_LNA                      0x2F30
#define CC120X_RXMIX                    0x2F31
#define CC120X_XOSC5                    0x2F32
#define CC120X_XOSC4                    0x2F33
#define CC120X_XOSC3                    0x2F34
#define CC120X_XOSC2                    0x2F35
#define CC120X_XOSC1                    0x2F36
#define CC120X_XOSC0                    0x2F37
#define CC120X_ANALOG_SPARE             0x2F38
#define CC120X_PA_CFG3                  0x2F39
#define CC120X_IRQ0M                    0x2F3F
#define CC120X_IRQ0F                    0x2F40  
  
/* Status Registers */
#define CC120X_WOR_TIME1                0x2F64
#define CC120X_WOR_TIME0                0x2F65
#define CC120X_WOR_CAPTURE1             0x2F66
#define CC120X_WOR_CAPTURE0             0x2F67
#define CC120X_BIST                     0x2F68
#define CC120X_DCFILTOFFSET_I1          0x2F69
#define CC120X_DCFILTOFFSET_I0          0x2F6A
#define CC120X_DCFILTOFFSET_Q1          0x2F6B
#define CC120X_DCFILTOFFSET_Q0          0x2F6C
#define CC120X_IQIE_I1                  0x2F6D
#define CC120X_IQIE_I0                  0x2F6E
#define CC120X_IQIE_Q1                  0x2F6F
#define CC120X_IQIE_Q0                  0x2F70
#define CC120X_RSSI1                    0x2F71
#define CC120X_RSSI0                    0x2F72
#define CC120X_MARCSTATE                0x2F73
#define CC120X_LQI_VAL                  0x2F74
#define CC120X_PQT_SYNC_ERR             0x2F75
#define CC120X_DEM_STATUS               0x2F76
#define CC120X_FREQOFF_EST1             0x2F77
#define CC120X_FREQOFF_EST0             0x2F78
#define CC120X_AGC_GAIN3                0x2F79
#define CC120X_AGC_GAIN2                0x2F7A
#define CC120X_AGC_GAIN1                0x2F7B
#define CC120X_AGC_GAIN0                0x2F7C
#define CC120X_CFM_RX_DATA_OUT         0x2F7D
#define CC120X_CFM_TX_DATA_IN          0x2F7E
#define CC120X_ASK_SOFT_RX_DATA         0x2F7F
#define CC120X_RNDGEN                   0x2F80
#define CC120X_MAGN2                    0x2F81
#define CC120X_MAGN1                    0x2F82
#define CC120X_MAGN0                    0x2F83
#define CC120X_ANG1                     0x2F84
#define CC120X_ANG0                     0x2F85
#define CC120X_CHFILT_I2                0x2F86
#define CC120X_CHFILT_I1                0x2F87
#define CC120X_CHFILT_I0                0x2F88
#define CC120X_CHFILT_Q2                0x2F89
#define CC120X_CHFILT_Q1                0x2F8A
#define CC120X_CHFILT_Q0                0x2F8B
#define CC120X_GPIO_STATUS              0x2F8C
#define CC120X_FSCAL_CTRL               0x2F8D
#define CC120X_PHASE_ADJUST             0x2F8E
#define CC120X_PARTNUMBER               0x2F8F
#define CC120X_PARTVERSION              0x2F90
#define CC120X_SERIAL_STATUS            0x2F91
#define CC120X_MODEM_STATUS1            0x2F92
#define CC120X_MODEM_STATUS0            0x2F93
#define CC120X_MARC_STATUS1             0x2F94
#define CC120X_MARC_STATUS0             0x2F95
#define CC120X_PA_IFAMP_TEST            0x2F96
#define CC120X_FSRF_TEST                0x2F97
#define CC120X_PRE_TEST                 0x2F98
#define CC120X_PRE_OVR                  0x2F99
#define CC120X_ADC_TEST                 0x2F9A
#define CC120X_DVC_TEST                 0x2F9B
#define CC120X_ATEST                    0x2F9C
#define CC120X_ATEST_LVDS               0x2F9D
#define CC120X_ATEST_MODE               0x2F9E
#define CC120X_XOSC_TEST1               0x2F9F
#define CC120X_XOSC_TEST0               0x2FA0
#define CC120X_AES                      0x2FA1
#define CC120X_MDM_TEST                 0x2FA2  

#define CC120X_RXFIRST                  0x2FD2   
#define CC120X_TXFIRST                  0x2FD3   
#define CC120X_RXLAST                   0x2FD4 
#define CC120X_TXLAST                   0x2FD5 
#define CC120X_NUM_TXBYTES              0x2FD6  /* Number of bytes in TXFIFO */ 
#define CC120X_NUM_RXBYTES              0x2FD7  /* Number of bytes in RXFIFO */
#define CC120X_FIFO_NUM_TXBYTES         0x2FD8  
#define CC120X_FIFO_NUM_RXBYTES         0x2FD9  
#define CC120X_RXFIFO_PRE_BUF           0x2FDA
  
/* DATA FIFO Access */
#define CC120X_SINGLE_TXFIFO            0x003F     /*  TXFIFO  - Single accecss to Transmit FIFO */
#define CC120X_BURST_TXFIFO             0x007F     /*  TXFIFO  - Burst accecss to Transmit FIFO  */
#define CC120X_SINGLE_RXFIFO            0x00BF     /*  RXFIFO  - Single accecss to Receive FIFO  */
#define CC120X_BURST_RXFIFO             0x00FF     /*  RXFIFO  - Busrrst ccecss to Receive FIFO  */
  
/* AES Workspace */
/* AES Key */
#define CC120X_AES_KEY                  0x2FE0     /*  AES_KEY    - Address for AES key input  */
#define CC120X_AES_KEY15	        0x2FE0
#define CC120X_AES_KEY14	        0x2FE1
#define CC120X_AES_KEY13	        0x2FE2
#define CC120X_AES_KEY12	        0x2FE3
#define CC120X_AES_KEY11	        0x2FE4
#define CC120X_AES_KEY10	        0x2FE5
#define CC120X_AES_KEY9	                0x2FE6
#define CC120X_AES_KEY8	                0x2FE7
#define CC120X_AES_KEY7	                0x2FE8
#define CC120X_AES_KEY6	                0x2FE9
#define CC120X_AES_KEY5	                0x2FE10
#define CC120X_AES_KEY4	                0x2FE11
#define CC120X_AES_KEY3	                0x2FE12
#define CC120X_AES_KEY2	                0x2FE13
#define CC120X_AES_KEY1	                0x2FE14
#define CC120X_AES_KEY0	                0x2FE15

/* AES Buffer */
#define CC120X_AES_BUFFER               0x2FF0     /*  AES_BUFFER - Address for AES Buffer     */ 
#define CC120X_AES_BUFFER15		0x2FF0
#define CC120X_AES_BUFFER14		0x2FF1
#define CC120X_AES_BUFFER13		0x2FF2
#define CC120X_AES_BUFFER12		0x2FF3
#define CC120X_AES_BUFFER11		0x2FF4
#define CC120X_AES_BUFFER10		0x2FF5
#define CC120X_AES_BUFFER9		0x2FF6
#define CC120X_AES_BUFFER8		0x2FF7
#define CC120X_AES_BUFFER7		0x2FF8
#define CC120X_AES_BUFFER6		0x2FF9
#define CC120X_AES_BUFFER5		0x2FF10
#define CC120X_AES_BUFFER4		0x2FF11
#define CC120X_AES_BUFFER3		0x2FF12
#define CC120X_AES_BUFFER2		0x2FF13
#define CC120X_AES_BUFFER1		0x2FF14
#define CC120X_AES_BUFFER0		0x2FF15

#define CC120X_LQI_CRC_OK_BM            0x80
#define CC120X_LQI_EST_BM               0x7F

/* Command strobe registers */
#define CC120X_SRES                     0x30      /*  SRES    - Reset chip. */
#define CC120X_SFSTXON                  0x31      /*  SFSTXON - Enable and calibrate frequency synthesizer. */
#define CC120X_SXOFF                    0x32      /*  SXOFF   - Turn off crystal oscillator. */
#define CC120X_SCAL                     0x33      /*  SCAL    - Calibrate frequency synthesizer and turn it off. */
#define CC120X_SRX                      0x34      /*  SRX     - Enable RX. Perform calibration if enabled. */
#define CC120X_STX                      0x35      /*  STX     - Enable TX. If in RX state, only enable TX if CCA passes. */
#define CC120X_SIDLE                    0x36      /*  SIDLE   - Exit RX / TX, turn off frequency synthesizer. */
#define CC120X_SAFC                     0x37      /*  AFC     - Automatic Frequency Correction */    
#define CC120X_SWOR                     0x38      /*  SWOR    - Start automatic RX polling sequence (Wake-on-Radio) */
#define CC120X_SPWD                     0x39      /*  SPWD    - Enter power down mode when CSn goes high. */
#define CC120X_SFRX                     0x3A      /*  SFRX    - Flush the RX FIFO buffer. */
#define CC120X_SFTX                     0x3B      /*  SFTX    - Flush the TX FIFO buffer. */
#define CC120X_SWORRST                  0x3C      /*  SWORRST - Reset real time clock. */
#define CC120X_SNOP                     0x3D      /*  SNOP    - No operation. Returns status byte. */
  
/* Chip states returned in status byte */
#define CC120X_STATE_IDLE               0x00
#define CC120X_STATE_RX                 0x10
#define CC120X_STATE_TX                 0x20
#define CC120X_STATE_FSTXON             0x30
#define CC120X_STATE_CALIBRATE          0x40
#define CC120X_STATE_SETTLING           0x50
#define CC120X_STATE_RXFIFO_ERROR       0x60
#define CC120X_STATE_TXFIFO_ERROR       0x70
  
/******************************************************************************
 * PROTPTYPES
 */ 

  
  //MUST REDEFINE FOR CC120x
/* basic set of access functions */
rfStatus_t cc120xSpiReadReg(uint16 addr, uint8 *data, uint8 len);
rfStatus_t cc120xGetTxStatus(void);
rfStatus_t cc120xGetRxStatus(void);  
rfStatus_t cc120xSpiWriteReg(uint16 addr, uint8 *data, uint8 len);
rfStatus_t cc120xSpiWriteTxFifo(uint8 *pWriteData, uint8 len);
rfStatus_t cc120xSpiReadRxFifo(uint8 *pReadData, uint8 len);

#ifdef  __cplusplus
}
#endif
#endif// CC120X_SPI_H