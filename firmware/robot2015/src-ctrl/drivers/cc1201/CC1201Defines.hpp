#pragma once

//CHIP INFO
#define CC1201_EXPECTED_PART_NUMBER 0x21

//EXTENDED ACCESS BYTE
#define CC1201_EXTENDED_ACCESS 		 0x2F

//ACCESS MODIFIERS (pg 11/114)
#define CC1201_BURST		0x40
#define CC1201_READ 	    0x80


//REGISTERS (BURST AND SINGLE) (pg 11/114)
#define CC1201_IOCFG3           0x00
#define CC1201_IOCFG2           0x01
#define CC1201_IOCFG1           0x02
#define CC1201_IOCFG0           0x03
#define CC1201_SYNC3            0x04
#define CC1201_SYNC2            0x05
#define CC1201_SYNC1            0x06
#define CC1201_SYNC0            0x07
#define CC1201_SYNC_CFG1        0x08
#define CC1201_SYNC_CFG0        0x09
#define CC1201_DEVIATION_M      0x0A
#define CC1201_MODCFG_DEV_E     0x0B
#define CC1201_DCFILT_CFG       0x0C
#define CC1201_PREAMBLE_CFG1    0x0D
#define CC1201_PREAMBLE_CFG0    0x0E
#define CC1201_IQIC             0x0F
#define CC1201_CHAN_BW          0x10
#define CC1201_MDMCFG1          0x11
#define CC1201_MDMCFG0          0x12
#define CC1201_SYMBOL_RATE2     0x13
#define CC1201_SYMBOL_RATE1     0x14
#define CC1201_SYMBOL_RATE0     0x15
#define CC1201_AGC_REF          0x16
#define CC1201_AGC_CS_THR       0x17
#define CC1201_AGC_GAIN_ADJUST  0x18
#define CC1201_AGC_CFG3         0x19
#define CC1201_AGC_CFG2         0x1A
#define CC1201_AGC_CFG1         0x1B
#define CC1201_AGC_CFG0         0x1C
#define CC1201_FIFO_CFG         0x1D
#define CC1201_DEV_ADDR         0x1E
#define CC1201_SETTLING_CFG     0x1F
#define CC1201_FS_CFG           0x20
#define CC1201_WOR_CFG1         0x21
#define CC1201_WOR_CFG0         0x22
#define CC1201_WOR_EVENT0_MSB   0x23
#define CC1201_WOR_EVENT0_LSB   0x24
#define CC1201_RXDCM_TIME       0x25
#define CC1201_PKT_CFG2         0x26
#define CC1201_PKT_CFG1         0x27
#define CC1201_PKT_CFG0         0x28
#define CC1201_RFEND_CFG1       0x29
#define CC1201_RFEND_CFG0       0x2A
#define CC1201_PA_CFG1          0x2B
#define CC1201_PA_CFG0          0x2C
#define CC1201_ASK_CFG          0x2D
#define CC1201_PKT_LEN          0x2E
//#define CC1201_EXTENDED_ADDRESS 0x2F
// COMMAND STROBES
#define CC1201_SRES             0x30
#define CC1201_SFSTXON          0x31
#define CC1201_SXOFF            0x32
#define CC1201_SCAL             0x33
#define CC1201_SRX              0x34
#define CC1201_STX              0x35
#define CC1201_SIDLE            0x36
#define CC1201_SAFC             0x37
#define CC1201_SWOR             0x38
#define CC1201_SPWD             0x39
#define CC1201_SFRX             0x3A
#define CC1201_SFTX             0x3B
#define CC1201_SWORRST          0x3C
#define CC1201_SNOP             0x3D


#define CC1201_DMA              0x3E
#define CC1201_TX_FIFO          0x3F
#define CC1201_RX_FIFO          0x3F

//EXTENDED REGISTERS (pg 12-13/114)
#define CC1201EXT_IF_MIX_CFG       0x00
#define CC1201EXT_FREQOFF_CFG      0x01
#define CC1201EXT_TOC_CFG          0x02
#define CC1201EXT_MARC_SPARE       0x03
#define CC1201EXT_ECG_CFG          0x04
#define CC1201EXT_MDMCFG2          0x05
#define CC1201EXT_EXT_CTRL         0x06
#define CC1201EXT_RCCAL_FINE       0x07
#define CC1201EXT_RCCAL_COARSE     0x08
#define CC1201EXT_RCCAL_OFFSET     0x09
#define CC1201EXT_FREQOFF1         0x0A
#define CC1201EXT_FREQOFF0         0x0B
#define CC1201EXT_FREQ2            0x0C
#define CC1201EXT_FREQ1            0x0D
#define CC1201EXT_FREQ0            0x0E
#define CC1201EXT_IF_ADC2          0x0F
#define CC1201EXT_IF_ADC1          0x10
#define CC1201EXT_IF_ADC0          0x11
#define CC1201EXT_FS_DIG1          0x12
#define CC1201EXT_FS_DIG0          0x13
#define CC1201EXT_FS_CAL3          0x14
#define CC1201EXT_FS_CAL2          0x15
#define CC1201EXT_FS_CAL1          0x16
#define CC1201EXT_FS_CAL0          0x17
#define CC1201EXT_FS_CHP           0x18
#define CC1201EXT_FS_DIVTWO        0x19
#define CC1201EXT_FS_DSM1          0x1A
#define CC1201EXT_FS_DMS0          0x1B
#define CC1201EXT_FS_DVC1          0x1C
#define CC1201EXT_FS_DVC0          0x1D
#define CC1201EXT_FS_LBI           0x1E
#define CC1201EXT_FS_PFD           0x1F
#define CC1201EXT_FS_PRE           0x20
#define CC1201EXT_FS_REG_DIV_CML   0x21
#define CC1201EXT_FS_SPARE         0x22
#define CC1201EXT_FS_VCO4          0x23
#define CC1201EXT_FS_VCO3          0x24
#define CC1201EXT_FS_VCO2          0x25
#define CC1201EXT_FS_VCO1          0x26
#define CC1201EXT_FS_VCO0          0x27
#define CC1201EXT_GBIAS6           0x28
#define CC1201EXT_GBIAS5           0x29
#define CC1201EXT_GBIAS4           0x2A
#define CC1201EXT_GBIAS3           0x2B
#define CC1201EXT_GBIAS2           0x2C
#define CC1201EXT_GBIAS1           0x2D
#define CC1201EXT_GBIAS0           0x2E
#define CC1201EXT_IFAMP            0x2F
#define CC1201EXT_LNA              0x30
#define CC1201EXT_RXMIX            0x31
#define CC1201EXT_XOSC5            0x32
#define CC1201EXT_XOSC4            0x33
#define CC1201EXT_XOSC3            0x34
#define CC1201EXT_XOSC2            0x35
#define CC1201EXT_XOSC1            0x36
#define CC1201EXT_XOSC0            0x37
#define CC1201EXT_ANALOG_SPARE     0x38
#define CC1201EXT_PA_CFG3          0x39
//0x3A-0x3E NOT USED
//0x3F-0x40 RESERVED
//0x41-0x63 NOT USED
#define CC1201EXT_WOR_TIME1        0x64
#define CC1201EXT_WOR_TIME0        0x65
#define CC1201EXT_WOR_CAPTURE1     0x66
#define CC1201EXT_WOR_CAPTURE0     0x67
#define CC1201EXT_BIST             0x68
#define CC1201EXT_DCFILTOFFSET_I1  0x69
#define CC1201EXT_DCFILTOFFSET_T0  0x6A
#define CC1201EXT_DCFILTOFFSET_Q1  0x6B
#define CC1201EXT_DCFILTOFFSET_Q0  0x6C
#define CC1201EXT_IQIE_I1          0x6D
#define CC1201EXT_IQIE_I0          0x6E
#define CC1201EXT_IQIE_Q1          0x6F
#define CC1201EXT_IQIE_Q0          0x70
#define CC1201EXT_RSSI1            0x71
#define CC1201EXT_RSSI0            0x72
#define CC1201EXT_MARCSTATE        0x73
#define CC1201EXT_LQI_VAL          0x74
#define CC1201EXT_PQT_SYNC_ERR     0x75
#define CC1201EXT_DEM_STATUS       0x76
#define CC1201EXT_FREQOFF_EST1     0x77
#define CC1201EXT_FREQOFF_EST0     0x78
#define CC1201EXT_AGC_GAIN3        0x79
#define CC1201EXT_AGC_GAIN2        0x7A
#define CC1201EXT_AGC_GAIN1        0x7B
#define CC1201EXT_AGC_GAIN0        0x7C
#define CC1201EXT_CFM_RX_DATA_OUT  0x7D
#define CC1201EXT_CFM_RX_DATA_IN   0x7E
#define CC1201EXT_ASK_SOFT_RX_DATA 0x7F
#define CC1201EXT_RNDGEN           0x80
#define CC1201EXT_MAGN2            0x81
#define CC1201EXT_MAGN1            0x82
#define CC1201EXT_MAGN0            0x83
#define CC1201EXT_ANG1             0x84
#define CC1201EXT_ANG0             0x85
#define CC1201EXT_CHFILT_I2        0x86
#define CC1201EXT_CHFILT_I1        0x87
#define CC1201EXT_CHFILT_I0        0x88
#define CC1201EXT_CHFILT_Q2        0x89
#define CC1201EXT_CHFILT_Q1        0x8A
#define CC1201EXT_CHFILT_Q0        0x8B
#define CC1201EXT_GPIO_STATUS      0x8C
#define CC1201EXT_FSCAL_CTRL       0x8D
#define CC1201EXT_PHASE_ADJUST     0x8E
#define CC1201EXT_PARTNUMBER       0x8F
#define CC1201EXT_PARTVERSION      0x90
#define CC1201EXT_SERIAL_STATUS    0x91
#define CC1201EXT_MODEM_STATUS1    0x92
#define CC1201EXT_MODEM_STATUS0    0x93
#define CC1201EXT_MARC_STATUS1     0x94
#define CC1201EXT_MARC_STATUS0     0x95
#define CC1201EXT_PA_IFAMP_TEST    0x96
#define CC1201EXT_FSRF_TEST        0x97
#define CC1201EXT_PRE_TEST         0x98
#define CC1201EXT_PRE_OVR          0x99
#define CC1201EXT_ADC_TEST         0x9A
#define CC1201EXT_DVC_TEST         0x9B
#define CC1201EXT_ATEST            0x9C
#define CC1201EXT_ATEST_LVDS       0x9D
#define CC1201EXT_ATEST_MODE       0x9E
#define CC1201EXT_XOSC_TEST1       0x9F
#define CC1201EXT_XOSC_TEST0       0xA0
#define CC1201EXT_AES              0xA1
#define CC1201EXT_MDM_TEST         0xA2
//0xA3-0xD1 NOT USED
#define CC1201EXT_RXFIRST          0xD2
#define CC1201EXT_TXFIRST          0xD3
#define CC1201EXT_RXLAST           0xD4
#define CC1201EXT_TXLAST           0xD5
#define CC1201EXT_NUM_TXBYTES      0xD6
#define CC1201EXT_NUM_RXBYTES      0xD7
#define CC1201EXT_FIFO_NUM_TXBYTES 0xD8
#define CC1201EXT_FIFO_NUM_RXBYTES 0xD9
#define CC1201EXT_RXFIFO_PRE_BUF   0xDA
//0xDB-0xDF NOT USED
//0xE0-0xFF AES WORKSPACE

// STROBE COMMANDS (pg 13/114)
#define CC1201_STROBE_SRES    0x30        // Reset chip.
#define CC1201_STROBE_SFSTXON 0x31        // Enable and calibrate frequency synthesizer (if MCSM0.FS_AUTOCAL=1).
#define CC1201_STROBE_SXOFF   0x32        // Turn off crystal oscillator.
#define CC1201_STROBE_SCAL    0x33        // Calibrate frequency synthesizer and turn it off
#define CC1201_STROBE_SRX     0x34        // Enable RX. Perform calibration first if coming from IDLE and
#define CC1201_STROBE_STX     0x35        // In IDLE state: Enable TX. Perform calibration first if
#define CC1201_STROBE_SIDLE   0x36        // Exit RX / TX, turn off frequency synthesizer and exit
#define CC1201_STROBE_SAFC    0x37        // Perform AFC adjustment of the frequency synthesizer
#define CC1201_STROBE_SWOR    0x38        // Start automatic RX polling sequence (Wake-on-Radio)
#define CC1201_STROBE_SPWD    0x39        // Enter power down mode when CSn goes high.
#define CC1201_STROBE_SFRX    0x3A        // Flush the RX FIFO buffer.
#define CC1201_STROBE_SFTX    0x3B        // Flush the TX FIFO buffer.
#define CC1201_STROBE_SWORRST 0x3C        // Reset real time clock.
#define CC1201_STROBE_SNOP    0x3D        // nop

//DIRECT MEMORY ACCESS (DMA) (pg 14/114)
#define CC1201_DMA_TXFIFO_LOW  0x00
#define CC1201_DMA_TXFIFO_HIGH 0x7F
#define CC1201_DMA_RXFIFO_LOW  0x80
#define CC1201_DMA_RXFIFO_HIGH 0xFF

//TXOFF_MODE state transitions
#define TXOFF_MODE_IDLE   0x00
#define TXOFF_MODE_FSTXON 0x10
#define TXOFF_MODE_TX     0x20
#define TXOFF_MODE_RX     0x30

//RXOFF_MODE state transitions 
#define RXOFF_MODE_IDLE   0x00
#define RXOFF_MODE_FSTXON 0x10
#define RXOFF_MODE_TX     0x20
#define RXOFF_MODE_RX     0x30

#define CC1201_RX_FIFO_ERROR	0x60
#define CC1201_TX_FIFO_ERROR	0x70
