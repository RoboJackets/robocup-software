/***************************************************************
 *  SmartRF Studio(tm) Export
 *
 *  Radio register settings specifed with C-code
 *  compatible #define statements.
 *
 *  RF device: CC1201
 *
 ***************************************************************/

#ifndef SMARTRF_CC1201_H
#define SMARTRF_CC1201_H

#define SMARTRF_RADIO_CC1201

// GPIO configurations are [almost] completely independent from modulation implementations
#define SMARTRF_SETTING_IOCFG3             0x30	// HIGHZ
#define SMARTRF_SETTING_IOCFG2             0x30	// HIGHZ
#define SMARTRF_SETTING_IOCFG1             0x30	// HIGHZ [used as SO for SPI when CSn is LOW]
#define SMARTRF_SETTING_IOCFG0             0x07	// Asserted when PKT_CRC_OK is set. De-asserted at first byte read from the RX FIFO

// Default sync word
#define SMARTRF_SETTING_SYNC3              0x93
#define SMARTRF_SETTING_SYNC2              0x0B
#define SMARTRF_SETTING_SYNC1              0x51
#define SMARTRF_SETTING_SYNC0              0xDE


#define SMARTRF_SETTING_SYNC_CFG1          0xAF	// 0xA8
#define SMARTRF_SETTING_SYNC_CFG0          0x03	// Highest sensitivity here
#define SMARTRF_SETTING_DEVIATION_M        0x47
#define SMARTRF_SETTING_MODCFG_DEV_E       0x2F
#define SMARTRF_SETTING_DCFILT_CFG         0x1E
#define SMARTRF_SETTING_PREAMBLE_CFG1      0x16	// 0x14
#define SMARTRF_SETTING_PREAMBLE_CFG0      0x8A
#define SMARTRF_SETTING_IQIC               0x00
#define SMARTRF_SETTING_CHAN_BW            0x01
#define SMARTRF_SETTING_MDMCFG1            0x42
#define SMARTRF_SETTING_MDMCFG0            0x05
#define SMARTRF_SETTING_SYMBOL_RATE2       0xC9
#define SMARTRF_SETTING_SYMBOL_RATE1       0x99
#define SMARTRF_SETTING_SYMBOL_RATE0       0x99
#define SMARTRF_SETTING_AGC_REF            0x2F
#define SMARTRF_SETTING_AGC_CS_THR         0x01
#define SMARTRF_SETTING_AGC_GAIN_ADJUST    0x00
#define SMARTRF_SETTING_AGC_CFG3           0xB1	// 0xB1
#define SMARTRF_SETTING_AGC_CFG2           0x60
#define SMARTRF_SETTING_AGC_CFG1           0x12
#define SMARTRF_SETTING_AGC_CFG0           0x84
#define SMARTRF_SETTING_FIFO_CFG           0x80	// 0x00
#define SMARTRF_SETTING_DEV_ADDR           0x00
#define SMARTRF_SETTING_SETTLING_CFG       0x0B
#define SMARTRF_SETTING_FS_CFG             0x02	// 0x12
#define SMARTRF_SETTING_WOR_CFG1           0x08
#define SMARTRF_SETTING_WOR_CFG0           0x21
#define SMARTRF_SETTING_WOR_EVENT0_MSB     0x00
#define SMARTRF_SETTING_WOR_EVENT0_LSB     0x00
#define SMARTRF_SETTING_RXDCM_TIME         0x00
#define SMARTRF_SETTING_PKT_CFG2           0x04	// 0x00
#define SMARTRF_SETTING_PKT_CFG1           0x03
#define SMARTRF_SETTING_PKT_CFG0           0x20
#define SMARTRF_SETTING_RFEND_CFG1         0x0F
#define SMARTRF_SETTING_RFEND_CFG0         0x30	// 0x00
#define SMARTRF_SETTING_PA_CFG1            0x7F	// 14dBm output power
#define SMARTRF_SETTING_PA_CFG0            0x56
#define SMARTRF_SETTING_ASK_CFG            0x0F
#define SMARTRF_SETTING_PKT_LEN            0xFF
#define SMARTRF_SETTING_IF_MIX_CFG         0x00
#define SMARTRF_SETTING_FREQOFF_CFG        0x20	// 0x23
#define SMARTRF_SETTING_TOC_CFG            0x0B
#define SMARTRF_SETTING_MARC_SPARE         0x00
#define SMARTRF_SETTING_ECG_CFG            0x00
#define SMARTRF_SETTING_MDMCFG2            0x00
#define SMARTRF_SETTING_EXT_CTRL           0x01
#define SMARTRF_SETTING_RCCAL_FINE         0x00
#define SMARTRF_SETTING_RCCAL_COARSE       0x00
#define SMARTRF_SETTING_RCCAL_OFFSET       0x00
#define SMARTRF_SETTING_FREQOFF1           0x00
#define SMARTRF_SETTING_FREQOFF0           0x00

 /*	920.5999 MHz
#define SMARTRF_SETTING_FREQ2              0x5C
#define SMARTRF_SETTING_FREQ1              0x0F
#define SMARTRF_SETTING_FREQ0              0x5C
 */

// 915.99 MHz
#define SMARTRF_SETTING_FREQ2              0x5B
#define SMARTRF_SETTING_FREQ1              0x99
#define SMARTRF_SETTING_FREQ0              0x5C

#define SMARTRF_SETTING_IF_ADC2            0x02
#define SMARTRF_SETTING_IF_ADC1            0xEE
#define SMARTRF_SETTING_IF_ADC0            0x10
#define SMARTRF_SETTING_FS_DIG1            0x04
#define SMARTRF_SETTING_FS_DIG0            0x5F
#define SMARTRF_SETTING_FS_CAL3            0x00
#define SMARTRF_SETTING_FS_CAL2            0x20
#define SMARTRF_SETTING_FS_CAL1            0x40
#define SMARTRF_SETTING_FS_CAL0            0x00
#define SMARTRF_SETTING_FS_CHP             0x28
#define SMARTRF_SETTING_FS_DIVTWO          0x03
#define SMARTRF_SETTING_FS_DSM1            0x00
#define SMARTRF_SETTING_FS_DSM0            0x33
#define SMARTRF_SETTING_FS_DVC1            0xFF
#define SMARTRF_SETTING_FS_DVC0            0x17
#define SMARTRF_SETTING_FS_LBI             0x00
#define SMARTRF_SETTING_FS_PFD             0x00
#define SMARTRF_SETTING_FS_PRE             0x6E
#define SMARTRF_SETTING_FS_REG_DIV_CML     0x1C
#define SMARTRF_SETTING_FS_SPARE           0xAC
#define SMARTRF_SETTING_FS_VCO4            0x14
#define SMARTRF_SETTING_FS_VCO3            0x00
#define SMARTRF_SETTING_FS_VCO2            0x00
#define SMARTRF_SETTING_FS_VCO1            0x00
#define SMARTRF_SETTING_FS_VCO0            0xB5
#define SMARTRF_SETTING_GBIAS6             0x00
#define SMARTRF_SETTING_GBIAS5             0x02
#define SMARTRF_SETTING_GBIAS4             0x00
#define SMARTRF_SETTING_GBIAS3             0x00
#define SMARTRF_SETTING_GBIAS2             0x10
#define SMARTRF_SETTING_GBIAS1             0x00
#define SMARTRF_SETTING_GBIAS0             0x00
#define SMARTRF_SETTING_IFAMP              0x0D
#define SMARTRF_SETTING_LNA                0x01
#define SMARTRF_SETTING_RXMIX              0x01
#define SMARTRF_SETTING_XOSC5              0x0E
#define SMARTRF_SETTING_XOSC4              0xA0
#define SMARTRF_SETTING_XOSC3              0x03
#define SMARTRF_SETTING_XOSC2              0x05	// 0x04
#define SMARTRF_SETTING_XOSC1              0x03
#define SMARTRF_SETTING_XOSC0              0x00
#define SMARTRF_SETTING_ANALOG_SPARE       0x00
#define SMARTRF_SETTING_PA_CFG3            0x00
#define SMARTRF_SETTING_WOR_TIME1          0x00
#define SMARTRF_SETTING_WOR_TIME0          0x00
#define SMARTRF_SETTING_WOR_CAPTURE1       0x00
#define SMARTRF_SETTING_WOR_CAPTURE0       0x00
#define SMARTRF_SETTING_BIST               0x00
#define SMARTRF_SETTING_DCFILTOFFSET_I1    0x00
#define SMARTRF_SETTING_DCFILTOFFSET_I0    0x00
#define SMARTRF_SETTING_DCFILTOFFSET_Q1    0x00
#define SMARTRF_SETTING_DCFILTOFFSET_Q0    0x00
#define SMARTRF_SETTING_IQIE_I1            0x00
#define SMARTRF_SETTING_IQIE_I0            0x00
#define SMARTRF_SETTING_IQIE_Q1            0x00
#define SMARTRF_SETTING_IQIE_Q0            0x00
#define SMARTRF_SETTING_RSSI1              0x80
#define SMARTRF_SETTING_RSSI0              0x00
#define SMARTRF_SETTING_MARCSTATE          0x41
#define SMARTRF_SETTING_LQI_VAL            0x00
#define SMARTRF_SETTING_PQT_SYNC_ERR       0xFF
#define SMARTRF_SETTING_DEM_STATUS         0x00
#define SMARTRF_SETTING_FREQOFF_EST1       0x00
#define SMARTRF_SETTING_FREQOFF_EST0       0x00
#define SMARTRF_SETTING_AGC_GAIN3          0x00
#define SMARTRF_SETTING_AGC_GAIN2          0xD1
#define SMARTRF_SETTING_AGC_GAIN1          0x00
#define SMARTRF_SETTING_AGC_GAIN0          0x3F
#define SMARTRF_SETTING_CFM_RX_DATA_OUT    0x00
#define SMARTRF_SETTING_CFM_TX_DATA_IN     0x00
#define SMARTRF_SETTING_ASK_SOFT_RX_DATA   0x30
#define SMARTRF_SETTING_RNDGEN             0xFF	// 0x7F
#define SMARTRF_SETTING_MAGN2              0x00
#define SMARTRF_SETTING_MAGN1              0x00
#define SMARTRF_SETTING_MAGN0              0x00
#define SMARTRF_SETTING_ANG1               0x00
#define SMARTRF_SETTING_ANG0               0x00
#define SMARTRF_SETTING_CHFILT_I2          0x02
#define SMARTRF_SETTING_CHFILT_I1          0x00
#define SMARTRF_SETTING_CHFILT_I0          0x00
#define SMARTRF_SETTING_CHFILT_Q2          0x00
#define SMARTRF_SETTING_CHFILT_Q1          0x00
#define SMARTRF_SETTING_CHFILT_Q0          0x00
#define SMARTRF_SETTING_GPIO_STATUS        0x00
#define SMARTRF_SETTING_FSCAL_CTRL         0x01
#define SMARTRF_SETTING_PHASE_ADJUST       0x00
#define SMARTRF_SETTING_PARTNUMBER         0x00
#define SMARTRF_SETTING_PARTVERSION        0x00
#define SMARTRF_SETTING_SERIAL_STATUS      0x00
#define SMARTRF_SETTING_MODEM_STATUS1      0x01
#define SMARTRF_SETTING_MODEM_STATUS0      0x00
#define SMARTRF_SETTING_MARC_STATUS1       0x00
#define SMARTRF_SETTING_MARC_STATUS0       0x00
#define SMARTRF_SETTING_PA_IFAMP_TEST      0x00
#define SMARTRF_SETTING_FSRF_TEST          0x00
#define SMARTRF_SETTING_PRE_TEST           0x00
#define SMARTRF_SETTING_PRE_OVR            0x00
#define SMARTRF_SETTING_ADC_TEST           0x00
#define SMARTRF_SETTING_DVC_TEST           0x0B
#define SMARTRF_SETTING_ATEST              0x40
#define SMARTRF_SETTING_ATEST_LVDS         0x00
#define SMARTRF_SETTING_ATEST_MODE         0x00
#define SMARTRF_SETTING_XOSC_TEST1         0x3C
#define SMARTRF_SETTING_XOSC_TEST0         0x00
#define SMARTRF_SETTING_AES                0x00
#define SMARTRF_SETTING_MDM_TEST           0x00
#define SMARTRF_SETTING_RXFIRST            0x00
#define SMARTRF_SETTING_TXFIRST            0x00
#define SMARTRF_SETTING_RXLAST             0x00
#define SMARTRF_SETTING_TXLAST             0x00
#define SMARTRF_SETTING_NUM_TXBYTES        0x00
#define SMARTRF_SETTING_NUM_RXBYTES        0x00
#define SMARTRF_SETTING_FIFO_NUM_TXBYTES   0x0F
#define SMARTRF_SETTING_FIFO_NUM_RXBYTES   0x00
#define SMARTRF_SETTING_RXFIFO_PRE_BUF     0x00
#define SMARTRF_SETTING_AES_KEY15          0x00
#define SMARTRF_SETTING_AES_KEY14          0x00
#define SMARTRF_SETTING_AES_KEY13          0x00
#define SMARTRF_SETTING_AES_KEY12          0x00
#define SMARTRF_SETTING_AES_KEY11          0x00
#define SMARTRF_SETTING_AES_KEY10          0x00
#define SMARTRF_SETTING_AES_KEY9           0x00
#define SMARTRF_SETTING_AES_KEY8           0x00
#define SMARTRF_SETTING_AES_KEY7           0x00
#define SMARTRF_SETTING_AES_KEY6           0x00
#define SMARTRF_SETTING_AES_KEY5           0x00
#define SMARTRF_SETTING_AES_KEY4           0x00
#define SMARTRF_SETTING_AES_KEY3           0x00
#define SMARTRF_SETTING_AES_KEY2           0x00
#define SMARTRF_SETTING_AES_KEY1           0x00
#define SMARTRF_SETTING_AES_KEY0           0x00
#define SMARTRF_SETTING_AES_BUFFER15       0x00
#define SMARTRF_SETTING_AES_BUFFER14       0x00
#define SMARTRF_SETTING_AES_BUFFER13       0x00
#define SMARTRF_SETTING_AES_BUFFER12       0x00
#define SMARTRF_SETTING_AES_BUFFER11       0x00
#define SMARTRF_SETTING_AES_BUFFER10       0x00
#define SMARTRF_SETTING_AES_BUFFER9        0x00
#define SMARTRF_SETTING_AES_BUFFER8        0x00
#define SMARTRF_SETTING_AES_BUFFER7        0x00
#define SMARTRF_SETTING_AES_BUFFER6        0x00
#define SMARTRF_SETTING_AES_BUFFER5        0x00
#define SMARTRF_SETTING_AES_BUFFER4        0x00
#define SMARTRF_SETTING_AES_BUFFER3        0x00
#define SMARTRF_SETTING_AES_BUFFER2        0x00
#define SMARTRF_SETTING_AES_BUFFER1        0x00
#define SMARTRF_SETTING_AES_BUFFER0        0x00

#endif
